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
#include "RandomRepair.h"
#include "RegretRepair.h"
#include "Repairer.h"
#include "Solution.h"
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
    std::cout << "  -f \"folder of instance/drive time/ public transport time\" | Optional  | Default \"./\"\n";
    std::cout << "  -v \"number of vehicles\"                                   | Optional  | Default nbRequests-1\n";
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
    const char* mandatory[] = {"-p", "-w", "-d", "-pt", "-c", "-ci"};
    for (const char* flag : mandatory) {
        if (getOptionValue(argc, argv, flag) == nullptr) return false;
    }
    return true;
}

// Fresh operator instances each time -- both PartialLNS and LNS own their own
// list (unique_ptr), so the two stages can't share one set.
std::vector<std::unique_ptr<Destroyer>> makeDestroyers() {
    std::vector<std::unique_ptr<Destroyer>> destroyers;
    destroyers.push_back(std::make_unique<RandomRemoval>());
    // TODO: related/worst/route/cluster/separateSelection removal, once built.
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
    instance.precomputeAccessOptions();
    instance.setNbVehicles(instance.nbRequests());

    if (getOptionValue(argc, argv, "-v") != nullptr) {
        int valeur;
        std::ifstream(getOptionValue(argc, argv, "-v")) >> valeur;
        instance.setNbVehicles(valeur+1);
    } 
    // Stage 1: build an initial solution from empty routes, tolerating
    // unassigned requests along the way. Uses conf.timeLimit() as its own
    // budget too (separate from stage 2's, measured from its own start) --
    // the old code used a distinct, much larger hardcoded budget (3600s) for
    // this stage; if you want that split again, add a dedicated field to
    // ConfLNS rather than reusing timeLimit() for both.
    //
    // NOTE: -sinit (loading a pre-built initial solution from file) isn't
    // wired in -- Solution has no file reader yet.
    TimePoint start = now();
    PartialLNS partialLns(conf, makeDestroyers(), makeRepairersRandom());
    Solution sInit = partialLns.compute(instance);
    std::cout << "Fin Sinit\n";
    sInit.computeObjectiveFunction();
    //std::cout << "Initial solution: " << sInit.unassignedRequests().size() << " unassigned request(s), objective "
    //          << sInit.objectiveFunction() << " (" << partialLns.lastIteration() << " iteration(s))\n";

    // Stage 2: improve it, requiring every request to stay assigned.
    //std::cout << "Optimizing...\n";
    LNS lns(conf, makeDestroyers(), makeRepairers());
    Solution best = lns.compute(sInit, conf.timeLimit()-secondsSince(start));
    best.computeObjectiveFunction();
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

