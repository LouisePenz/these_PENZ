#include "PartialLNS.h"

#include <algorithm>
#include <cstdlib>

#include "Clock.h"

PartialLNS::PartialLNS(const ConfLNS& conf, std::vector<std::unique_ptr<Destroyer>> destroyers,
                        std::vector<std::unique_ptr<Repairer>> repairers)
    : conf_(&conf), destroyers_(std::move(destroyers)), repairers_(std::move(repairers)) {}

Destroyer& PartialLNS::pickDestroyer() { return *destroyers_[rand() % destroyers_.size()]; }
Repairer& PartialLNS::pickRepairer() { return *repairers_[rand() % repairers_.size()]; }

long long PartialLNS::score(const Solution& solution) const {
    return unassignedWeight_ * static_cast<long long>(solution.unassignedRequests().size()) +
           solution.objectiveFunction();
}

Solution PartialLNS::compute(const Instance& instance, int noImprovementLimit, int timeLimit) {
    // Large enough that a single unassigned request always outweighs any
    // achievable difference in driving cost (bounded by roughly 2 * horizon
    // per vehicle: out and back within the horizon), regardless of instance size.
    if(timeLimit == -1)
        timeLimit = conf_->timeLimit();
    // The users' riding time (Objective::RidingTime) is bounded by nbRequests * horizon.
    unassignedWeight_ = (2LL * instance.nbVehicles() + instance.nbRequests()) * instance.horizonTime() + 1;

    Solution current(instance);
    current.computeObjectiveFunction();
    Solution best = current;

    int nbRequests = instance.nbRequests();
    int removeMin = std::min(conf_->nbRequestsRemovedMin(), nbRequests);
    int removeMax = std::min(conf_->nbRequestsRemovedMax(), nbRequests);
    if (removeMax < removeMin) removeMax = removeMin;

    int iterationsSinceChange = 0;
    iteration_ = 0;

    TimePoint start = now();
    while (secondsSince(start) < timeLimit && !best.unassignedRequests().empty()) {
        Solution trial = current;  // cheap: value types, no manual undo needed on rejection

        int nbToRemove = removeMin + (removeMax > removeMin ? rand() % (removeMax - removeMin + 1) : 0);
        pickDestroyer().apply(trial, nbToRemove);
        pickRepairer().apply(trial, /*allowPartial=*/true);
        trial.computeObjectiveFunction();

        if (score(trial) < score(current)) {
            current = trial;
            iterationsSinceChange = 0;
            if (score(current) < score(best)) best = current;
        } else {
            ++iterationsSinceChange;
        }

        if (iterationsSinceChange >= noImprovementLimit) {
            current = Solution(instance);  // back to empty routes, every request unassigned
            current.computeObjectiveFunction();
            iterationsSinceChange = 0;
        }

        ++iteration_;
    }

    return best;
}
