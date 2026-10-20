#pragma once

#include <memory>
#include <vector>

#include "ConfLNS.h"
#include "Destroyer.h"
#include "Repairer.h"
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
    // untouched) until conf.timeLimit() seconds have elapsed. Returns the
    // best solution found; if sInit already has unassigned requests, they
    // stay unassigned throughout (this class never places or removes a
    // request that repair itself doesn't touch).
    Solution compute(const Solution& sInit, int timeLimit = -1);

    int lastIteration() const noexcept { return iteration_; }
    int lastChangeIteration() const noexcept { return lastChangeIteration_; }

private:
    Destroyer& pickDestroyer();
    Repairer& pickRepairer();

    const ConfLNS* conf_;
    std::vector<std::unique_ptr<Destroyer>> destroyers_;
    std::vector<std::unique_ptr<Repairer>> repairers_;

    int iteration_ = 0;
    int lastChangeIteration_ = 0;
};
