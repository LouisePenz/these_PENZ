#pragma once

#include <iosfwd>
#include <memory>
#include <vector>

#include "ConfLNS.h"
#include "Destroyer.h"
#include "Repairer.h"
#include "RoutePool.h"
#include "Solution.h"

// Large Neighborhood Search that only ever accepts complete solutions: if a
// repair step can't place every request, that iteration is discarded (the
// repairer is called with allowPartial = false). See PartialLNS for the
// variant used to build an initial solution from empty routes, where some
// requests may be left unassigned.
//
// Acceptance follows record-to-record travel: a repaired solution is
// accepted if its objective is within (1 + recordToRecordValue) of the
// current one; recordToRecordValue is itself periodically tightened or
// loosened every conf.updateIteration() feasible iterations, based on how
// many of them were accepted (matches ConfLNS's upper/lowerBoundRRT/rateRRT).
class LNS {
public:
    // Takes ownership of the destroy/repair operators. Passing a shorter
    // list now and a longer one later, once more operators exist, requires
    // no change to LNS itself.
    LNS(const ConfLNS& conf, std::vector<std::unique_ptr<Destroyer>> destroyers,
        std::vector<std::unique_ptr<Repairer>> repairers);

    // Runs the search starting from sInit (copied -- sInit itself is left
    // untouched) until timeLimit seconds (conf.timeLimit() if -1) have
    // elapsed or conf.iterationLimit() iterations are done. Returns the
    // best solution found; if sInit already has unassigned requests, they
    // stay unassigned throughout (this class never places or removes a
    // request that repair itself doesn't touch).
    Solution compute(const Solution& sInit, int timeLimit = -1);

    int lastIteration() const noexcept { return iteration_; }
    int lastChangeIteration() const noexcept { return lastChangeIteration_; }
    // The routes of the complete solutions met by compute() -- sInit and
    // every feasible repaired solution, accepted or not -- since the last set
    // partitioning: each time the pool reaches
    // conf.setPartitioningPoolRatio() / (number of requests) routes, the set partitioning runs on it (within the time left); a
    // better solution becomes both the best and the current one; the pool is
    // then emptied, and starts again from the routes of the best solution (so
    // that the next run can start from it). What is left at the end is for a
    // last run by the caller.
    const RoutePool& pool() const noexcept { return pool_; }
    // Where the set partitioning runs report (none if null, the default).
    void setLog(std::ostream* log) noexcept { log_ = log; }
    // One line per set partitioning run, if not null (the default): the
    // number of routes, the best objective before the run and the objective
    // found by the run (NA if none). See writeSetPartitioningTrace.
    void setSetPartitioningTrace(std::ostream* trace) noexcept { spTrace_ = trace; }

private:
    Destroyer& pickDestroyer();
    Repairer& pickRepairer();

    const ConfLNS* conf_;
    std::vector<std::unique_ptr<Destroyer>> destroyers_;
    std::vector<std::unique_ptr<Repairer>> repairers_;

    int iteration_ = 0;
    int lastChangeIteration_ = 0;
    RoutePool pool_;
    std::ostream* log_ = nullptr;
    std::ostream* spTrace_ = nullptr;
};

// The trace line of one set partitioning run: "nbRoutes best recombined",
// recombined being NA if the run found no solution.
void writeSetPartitioningTrace(std::ostream& trace, int nbRoutes, int best, bool found, int recombined);
