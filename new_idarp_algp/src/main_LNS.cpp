#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ConfLNS.h"
#include "Destroyer.h"
#include "GreedyRepair.h"
#include "Instance.h"
#include "LNS.h"
#include "PartialLNS.h"
#include "RandomRemoval.h"
#include "RelatedRemoval.h"
#include "WorstRemoval.h"
#include "RandomRepair.h"
#include "RegretRepair.h"
#include "Repairer.h"
#include "SetPartitioning.h"
#include "Solution.h"
#include "SolutionChecker.h"
#include <Clock.h>

namespace {

void printHelp() {
    std::cout << "To run this program, please give:\n";
    std::cout << "  -c \"configuration file\"                                   | Mandatory |\n";
    std::cout << "  -ci \"configuration instance\"                              | Mandatory |\n";
    std::cout << "  -p \"file of the instance\"                                 | Mandatory |\n";
    std::cout << "  -d \"file of the drive time\"                               | Mandatory |\n";
    std::cout << "  -pt \"file of the public transport time\"                   | Mandatory |\n";
    std::cout << "  -w \"file of the walk time\"                                | Mandatory |\n";
    std::cout << "  -v \"file with the minimum number of vehicles\"             | Mandatory | (main_nb_vehicles -res); uses it + 1\n";
    std::cout << "  -f \"folder of instance/drive time/ public transport time\" | Optional  | Default \"./\"\n";
    std::cout << "  -s \"random seed\"                                          | Optional  | Default current time\n";
    std::cout << "  -sp \"file\"                                                | Optional  | One line per set partitioning\n";
    std::cout << "                                                            |           | run: nb routes, best before,\n";
    std::cout << "                                                            |           | recombined (NA if none)\n";
    std::cout << "  -check                                                    | Optional  | Check the initial and best\n";
    std::cout << "                                                            |           | solutions (SolutionChecker),\n";
    std::cout << "                                                            |           | report on stderr\n";
    std::cout << "\nNot yet supported by this rewrite (see comments in main_LNS.cpp):\n";
    std::cout << "  -sn / -sinit / -slns / -history\n";
}

bool hasFlag(int argc, char* argv[], const std::string& flag) {
    for (int i = 1; i < argc; ++i) {
        if (argv[i] == flag) return true;
    }
    return false;
}

// Returns nullptr if the flag is absent, or if it is the last argument (no value follows).
const char* getOptionValue(int argc, char* argv[], const std::string& flag) {
    for (int i = 1; i < argc; ++i) {
        if (flag == argv[i]) {
            return (i + 1 < argc) ? argv[i + 1] : nullptr;
        }
    }
    return nullptr;
}

bool hasAllMandatoryOptions(int argc, char* argv[]) {
    const char* mandatory[] = {"-p", "-w", "-d", "-pt", "-c", "-ci", "-v"};
    for (const char* flag : mandatory) {
        if (getOptionValue(argc, argv, flag) == nullptr) return false;
    }
    return true;
}

// Fresh operator instances each time -- both PartialLNS and LNS own their own
// list (unique_ptr), so the two stages can't share one set.
std::vector<std::unique_ptr<Destroyer>> makeDestroyers(const ConfLNS& conf) {
    std::vector<std::unique_ptr<Destroyer>> destroyers;
    destroyers.push_back(std::make_unique<RandomRemoval>());
    destroyers.push_back(std::make_unique<WorstRemoval>(conf.worstRemovalRandomness()));
    destroyers.push_back(std::make_unique<RelatedRemoval>(RelatedRemoval::Measure::PickupAndDelivery,
                                                          conf.relatedRemovalRandomness()));
    destroyers.push_back(std::make_unique<RelatedRemoval>(RelatedRemoval::Measure::AnyEnds,
                                                          conf.relatedRemovalRandomness()));
    // TODO: route/cluster removal, once built.
    return destroyers;
}

std::vector<std::unique_ptr<Repairer>> makeRepairers() {
    std::vector<std::unique_ptr<Repairer>> repairers;
    repairers.push_back(std::make_unique<RandomRepair>());
    repairers.push_back(std::make_unique<GreedyRepair>());
    repairers.push_back(std::make_unique<RegretRepair>());
    return repairers;
}

std::vector<std::unique_ptr<Repairer>> makeRepairersRandom() {
    std::vector<std::unique_ptr<Repairer>> repairers;
    repairers.push_back(std::make_unique<RandomRepair>());
    return repairers;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (hasFlag(argc, argv, "-h") || !hasAllMandatoryOptions(argc, argv)) {
        printHelp();
        return 0;
    }

    unsigned int seed = static_cast<unsigned int>(time(nullptr));

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-s" && i + 1 < argc) {
            seed = static_cast<unsigned int>(std::stoul(argv[i + 1]));
        }
    }

    srand(seed);
    std::string instanceFolder = "./";
    if (getOptionValue(argc, argv, "-f") != nullptr) {
        instanceFolder = std::string(getOptionValue(argc, argv, "-f"));
    }

    Instance instance(instanceFolder + getOptionValue(argc, argv, "-p"),
                       instanceFolder + getOptionValue(argc, argv, "-d"),
                       instanceFolder + getOptionValue(argc, argv, "-pt"),
                       instanceFolder + getOptionValue(argc, argv, "-w"));

    std::string searchConfigFile = getOptionValue(argc, argv, "-c");
    std::string instanceConfigFile = getOptionValue(argc, argv, "-ci");
    ConfLNS conf(searchConfigFile, instanceConfigFile);

    instance.configurePublicTransport(conf.waitingTime(), conf.minimumTimeInTransit());
    instance.configureWalk(conf.walkThreshold());
    instance.precomputeAccessOptions(conf.nbPtOptions());
    // -v: file holding the minimum number of vehicles found by main_nb_vehicles;
    // one more vehicle is used here so that a complete solution is found quickly.
    int minNbVehicles = 0;
    std::ifstream vehiclesFile(getOptionValue(argc, argv, "-v"));
    if (!(vehiclesFile >> minNbVehicles) || minNbVehicles <= 0) {
        std::cerr << "Cannot read a positive number of vehicles from " << getOptionValue(argc, argv, "-v") << "\n";
        return 1;
    }
    instance.setNbVehicles(minNbVehicles + 1);

    // Stage 1: build an initial solution from empty routes, tolerating
    // unassigned requests along the way. Stages 1 and 2 share the same
    // conf.timeLimit() budget, measured from here.
    //
    // NOTE: -sinit (loading a pre-built initial solution from file) isn't
    // wired in -- Solution has no file reader yet.
    TimePoint start = now();
    PartialLNS partialLns(conf, makeDestroyers(conf), makeRepairersRandom());
    Solution sInit = partialLns.compute(instance);
    sInit.computeObjectiveFunction();
    bool check = hasFlag(argc, argv, "-check");
    if (check) {
        std::cerr << "Initial solution:\n";
        checkSolution(sInit, std::cerr);
    }

    // An incomplete initial solution means the whole time budget is already
    // spent: no optimization, and a deliberately bad objective (worse than
    // any complete solution) so that irace ranks this run last.
    if (!sInit.unassignedRequests().empty()) {
        std::cerr << "No complete initial solution within the time limit (" << sInit.unassignedRequests().size()
                  << " unassigned request(s))\n";
        std::ofstream objectiveFile("objective.txt");
        objectiveFile << static_cast<long long>(instance.nbRequests()) * instance.horizonTime() << std::endl;
        return 0;
    }
    //std::cout << "Initial solution: " << sInit.unassignedRequests().size() << " unassigned request(s), objective "
    //          << sInit.objectiveFunction() << " (" << partialLns.lastIteration() << " iteration(s))\n";

    // Stage 2: improve it, requiring every request to stay assigned.
    //std::cout << "Optimizing...\n";
    LNS lns(conf, makeDestroyers(conf), makeRepairers());
    lns.setLog(check ? &std::cerr : nullptr);
    std::ofstream spTrace;
    if (getOptionValue(argc, argv, "-sp") != nullptr) {
        spTrace.open(getOptionValue(argc, argv, "-sp"));
        lns.setSetPartitioningTrace(&spTrace);
    }
    Solution best = lns.compute(sInit, conf.timeLimit()-secondsSince(start));
    best.computeObjectiveFunction();
    if (check) {
        std::cerr << "Best solution:\n";
        // It may come from a set partitioning run, with stop pairs outside
        // the requests' candidates.
        checkSolution(best, std::cerr, /*requireCandidatePairs=*/false);
    }

    // Stage 3: a last set partitioning over the rest of the routes -- those
    // collected since the LNS last ran it, plus the best solution's (no time
    // limit for now).
    Solution recombined(instance);
    bool found = solveSetPartitioning(lns.pool(), best, recombined, check ? &std::cerr : nullptr);
    if (spTrace.is_open())
        writeSetPartitioningTrace(spTrace, lns.pool().size(), best.objectiveFunction(), found,
                                  recombined.objectiveFunction());
    if (found) {
        if (check) {
            std::cerr << "Set partitioning solution:\n";
            // Its transfers may use stop pairs outside the requests' candidates.
            checkSolution(recombined, std::cerr, /*requireCandidatePairs=*/false);
        }
        if (recombined.objectiveFunction() < best.objectiveFunction()) best = recombined;
    }
    /*std::cout << "Best solution: " << best.unassignedRequests().size() << " unassigned request(s), objective "
              << best.objectiveFunction() << " (" << lns.lastIteration() << " iteration(s), last change at "
              << lns.lastChangeIteration() << ")\n";*/

    // NOTE: -slns (per-iteration step log) and -history (every accepted
    // solution) aren't wired in -- LNS doesn't record that history yet
    // (the old list_time/list_removal_operators/... bookkeeping was dropped
    // when LNS was rewritten; add it back to LNS if you want it here).
    /*if (getOptionValue(argc, argv, "-sres") != nullptr) {
        std::cout << "\n-sres isn't wired to a file writer yet (Solution has none) -- printing instead:\n";
        best.print();
    }*/

    std::ofstream objectiveFile("objective.txt");
    objectiveFile << best.objectiveFunction() << std::endl;

    return 0;
}

