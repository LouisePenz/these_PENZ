#include "LNS.h"

#include <cstdlib>
#include <ostream>

#include "Clock.h"
#include "SetPartitioning.h"

LNS::LNS(const ConfLNS& conf, std::vector<std::unique_ptr<Destroyer>> destroyers,
         std::vector<std::unique_ptr<Repairer>> repairers)
    : conf_(&conf), destroyers_(std::move(destroyers)), repairers_(std::move(repairers)) {}

Destroyer& LNS::pickDestroyer() { return *destroyers_[rand() % destroyers_.size()]; }
Repairer& LNS::pickRepairer() { return *repairers_[rand() % repairers_.size()]; }

Solution LNS::compute(const Solution& sInit, int timeLimit) {
    Solution current = sInit;
    current.computeObjectiveFunction();
    Solution best = current;
    // The set partitioning recombines legs with the constant PT times of the
    // matrix, and costs routes by their driving time: not with the tram
    // timetable, nor for the users' riding time (no pool collected either).
    const bool setPartitioning = !current.instance().travelTimes().hasTimetable() &&
                                 current.instance().objective() == Objective::RouteTime;
    if (setPartitioning && current.unassignedRequests().empty()) pool_.add(current);

    float recordToRecordValue = conf_->recordToRecordValue();
    int nbRequests = current.instance().nbRequests();
    int removeMin = nbRequests * conf_->percentRequestsRemovedMin() / 100;
    int removeMax = nbRequests * conf_->percentRequestsRemovedMax() / 100;
    if (removeMax < removeMin) removeMax = removeMin;
    int poolSizeForSetPartitioning = conf_->setPartitioningPoolRatio() / nbRequests;

    int nbIterationsAccepted = 0;
    int nbIterationsFeasible = 0;
    iteration_ = 0;
    lastChangeIteration_ = 0;
    if(timeLimit == -1)
        timeLimit = conf_->timeLimit();

    TimePoint start = now();
    while (secondsSince(start) < timeLimit && iteration_ < conf_->iterationLimit()) {
        Solution trial = current;  // cheap: value types, no manual undo needed on rejection

        int nbToRemove = removeMin + (removeMax > removeMin ? rand() % (removeMax - removeMin + 1) : 0);
        pickDestroyer().apply(trial, nbToRemove);

        bool feasible = pickRepairer().apply(trial, /*allowPartial=*/false);

        if (feasible) {
            trial.computeObjectiveFunction();
            ++nbIterationsFeasible;
            if (setPartitioning) pool_.add(trial);

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

        if (setPartitioning && pool_.size() >= poolSizeForSetPartitioning) {
            double timeLeft = timeLimit - secondsSince(start);
            Solution recombined(current.instance());
            if (timeLeft > 0) {
                bool found = solveSetPartitioning(pool_, best, recombined, log_, timeLeft);
                if (spTrace_)
                    writeSetPartitioningTrace(*spTrace_, pool_.size(), best.objectiveFunction(), found,
                                              recombined.objectiveFunction());
                if (found && recombined.objectiveFunction() < best.objectiveFunction()) {
                    best = recombined;
                    current = recombined;
                }
            }
            pool_.clear();
            pool_.add(best);
        }

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

void writeSetPartitioningTrace(std::ostream& trace, int nbRoutes, int best, bool found, int recombined) {
    trace << nbRoutes << " " << best << " ";
    if (found) trace << recombined;
    else trace << "NA";
    trace << std::endl;
}
