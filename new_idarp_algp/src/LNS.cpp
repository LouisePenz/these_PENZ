#include "LNS.h"

#include <cstdlib>

#include "Clock.h"

LNS::LNS(const ConfLNS& conf, std::vector<std::unique_ptr<Destroyer>> destroyers,
         std::vector<std::unique_ptr<Repairer>> repairers)
    : conf_(&conf), destroyers_(std::move(destroyers)), repairers_(std::move(repairers)) {}

Destroyer& LNS::pickDestroyer() { return *destroyers_[rand() % destroyers_.size()]; }
Repairer& LNS::pickRepairer() { return *repairers_[rand() % repairers_.size()]; }

Solution LNS::compute(const Solution& sInit, int timeLimit) {
    Solution current = sInit;
    current.computeObjectiveFunction();
    Solution best = current;

    float recordToRecordValue = conf_->recordToRecordValue();
    int nbRequests = current.instance().nbRequests();
    int removeMin = nbRequests * conf_->percentRequestsRemovedMin() / 100;
    int removeMax = nbRequests * conf_->percentRequestsRemovedMax() / 100;
    if (removeMax < removeMin) removeMax = removeMin;

    int nbIterationsAccepted = 0;
    int nbIterationsFeasible = 0;
    iteration_ = 0;
    lastChangeIteration_ = 0;
    if(timeLimit == -1)
        timeLimit = conf_->timeLimit();

    TimePoint start = now();
    while (secondsSince(start) < timeLimit) {
        Solution trial = current;  // cheap: value types, no manual undo needed on rejection

        int nbToRemove = removeMin + (removeMax > removeMin ? rand() % (removeMax - removeMin + 1) : 0);
        pickDestroyer().apply(trial, nbToRemove);

        bool feasible = pickRepairer().apply(trial, /*allowPartial=*/false);

        if (feasible) {
            trial.computeObjectiveFunction();
            ++nbIterationsFeasible;

            if (trial.objectiveFunction() <= current.objectiveFunction() * (1.0f + recordToRecordValue)) {
                ++nbIterationsAccepted;
                lastChangeIteration_ = iteration_;
                current = trial;
                if (current.objectiveFunction() <= best.objectiveFunction()) {
                    best = current;
                }
            }
            // Rejected: `trial` simply goes out of scope next iteration --
            // `current` was never touched, nothing to restore.
        }
        ++iteration_;

        if (conf_->updateIteration() > 0 && nbIterationsFeasible == conf_->updateIteration()) {
            int acceptedPercent = 100 * nbIterationsAccepted / conf_->updateIteration();
            if (acceptedPercent > conf_->upperBoundRRT()) {
                recordToRecordValue /= conf_->rateRRT();
            } else if (acceptedPercent < conf_->lowerBoundRRT()) {
                recordToRecordValue *= conf_->rateRRT();
            }
            nbIterationsAccepted = 0;
            nbIterationsFeasible = 0;
        }
    }

    return best;
}
