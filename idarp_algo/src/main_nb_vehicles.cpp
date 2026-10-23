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
    std::cout << "  -pt \"file of the public transport time\"                   | Mandatory | unless -tram\n";
    std::cout << "  -tram \"file of the tram timetable\"                        | Optional  | Replaces -pt: tram runs\n";
    std::cout << "                                                            |           | with their times (one tram\n";
    std::cout << "                                                            |           | per trip, no set partitioning)\n";
    std::cout << "  -w \"file of the walk time\"                                | Mandatory |\n";
    std::cout << "  -f \"folder of instance/drive time/ public transport time\" | Optional  | Default \"./\"\n";
    std::cout << "  -res \"file where to write the result\"                     | Optional  | Default do not write\n";
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
    const char* mandatory[] = {"-p", "-w", "-d", "-c", "-ci"};
    for (const char* flag : mandatory) {
        if (getOptionValue(argc, argv, flag) == nullptr) return false;
    }
    return getOptionValue(argc, argv, "-pt") != nullptr || getOptionValue(argc, argv, "-tram") != nullptr;
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
    //repairers.push_back(std::make_unique<GreedyRepair>());
    //repairers.push_back(std::make_unique<RegretRepair>());
    return repairers;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (hasFlag(argc, argv, "-h") || !hasAllMandatoryOptions(argc, argv)) {
        printHelp();
        return 0;
    }

    srand(static_cast<unsigned int>(time(nullptr)));

    std::string instanceFolder = "./";
    if (getOptionValue(argc, argv, "-f") != nullptr) {
        instanceFolder = std::string(getOptionValue(argc, argv, "-f"));
    }

    Instance instance(instanceFolder + getOptionValue(argc, argv, "-p"),
                       instanceFolder + getOptionValue(argc, argv, "-d"),
                       getOptionValue(argc, argv, "-pt") ? instanceFolder + getOptionValue(argc, argv, "-pt") : "",
                       instanceFolder + getOptionValue(argc, argv, "-w"));

    std::string searchConfigFile = getOptionValue(argc, argv, "-c");
    std::string instanceConfigFile = getOptionValue(argc, argv, "-ci");
    ConfLNS conf(searchConfigFile, instanceConfigFile);
    int nbVehicles = instance.nbRequests();

    if (getOptionValue(argc, argv, "-tram") != nullptr)
        instance.loadTramTimetable(instanceFolder + getOptionValue(argc, argv, "-tram"));
    instance.configurePublicTransport(conf.waitingTime(), conf.minimumTimeInTransit());
    instance.configureWalk(conf.walkThreshold());
    instance.precomputeAccessOptions(conf.nbPtOptions());
    instance.setNbVehicles(instance.nbRequests() - 1);

    int timeLimit = 10*60;
    TimePoint start = now();
    while (secondsSince(start) < timeLimit) {
        PartialLNS partialLns(conf, makeDestroyers(), makeRepairers());
        Solution sInit = partialLns.compute(instance, 50, timeLimit - secondsSince(start));
        if(sInit.unassignedRequests().size() == 0){
            nbVehicles = sInit.nbUsedRoutes();
            instance.setNbVehicles(nbVehicles - 1);
        }
    }

    if (getOptionValue(argc, argv, "-res") != nullptr) {
        std::ofstream objectiveFile(getOptionValue(argc, argv, "-res"));
        objectiveFile << nbVehicles << std::endl;
    } else {
        std::cout << nbVehicles << std::endl;
    }

    return 0;
}

