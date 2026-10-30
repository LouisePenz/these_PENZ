#include "LNS.h"

#include <algorithm>
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

    int nbRequests = current.instance().nbRequests();
    int removeMin = std::min(conf_->nbRequestsRemovedMin(), nbRequests);
    int removeMax = std::min(conf_->nbRequestsRemovedMax(), nbRequests);
    if (removeMax < removeMin) removeMax = removeMin;
    int poolSizeForSetPartitioning = conf_->setPartitioningPoolRatio() / nbRequests;

    // Record-to-record travel threshold (Santini, Ropke & Hvattum 2018):
    // decreases linearly from rrtStartThreshold to rrtEndThreshold over the
    // search, measured by the share of the time limit or of the iteration
    // limit already used, whichever is larger.
    const double startThreshold = conf_->rrtStartThreshold();
    const double endThreshold = conf_->rrtEndThreshold();
    double threshold = startThreshold;
    const int traceEvery = 100;  // feasible iterations per line of the RRT trace
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

            // Record-to-record travel: compared with the record (the best
            // solution), not with the current one, so that the current
            // solution cannot drift away step by step. Accepted if
            // (f(trial) - f(best)) / f(trial) < threshold.
            if (trial.objectiveFunction() - best.objectiveFunction() < threshold * trial.objectiveFunction()) {
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

        double progress = std::max(secondsSince(start) / timeLimit,
                                   static_cast<double>(iteration_) / conf_->iterationLimit());
        threshold = startThreshold - (startThreshold - endThreshold) * std::min(progress, 1.0);

        if (rrtTrace_ && nbIterationsFeasible == traceEvery) {
            *rrtTrace_ << iteration_ << " " << secondsSince(start) << " "
                       << 100 * nbIterationsAccepted / traceEvery << " " << threshold << " "
                       << current.objectiveFunction() << " " << best.objectiveFunction() << "\n";
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
