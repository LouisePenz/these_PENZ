#pragma once

#include <memory>
#include <vector>

#include "ConfLNS.h"
#include "Destroyer.h"
#include "Repairer.h"
#include "Instance.h"
#include "Solution.h"

// Builds a solution from empty routes, tolerating unassigned requests along
// the way (repairers are called with allowPartial = true). Meant to produce
// the initial solution LNS itself then improves -- not a general-purpose
// search.
//
// Acceptance is greedy on a lexicographic-style score: unassignedCount is
// weighted heavily enough to always dominate the objective (see the
// constructor), so coverage is improved first, driving cost second. If
// noImprovementLimit consecutive iterations produce no improvement, the
// current solution is discarded and rebuilt from empty routes (the best
// solution found so far is kept regardless -- only the search's current
// state resets).
class PartialLNS {
public:
    PartialLNS(const ConfLNS& conf, std::vector<std::unique_ptr<Destroyer>> destroyers,
               std::vector<std::unique_ptr<Repairer>> repairers);

    // Runs until every request is assigned or conf.timeLimit() seconds have
    // elapsed, restarting from empty routes after noImprovementLimit
    // consecutive non-improving iterations. Returns the best solution found
    // (which may still have unassigned requests if time ran out first).
    Solution compute(const Instance& instance, int noImprovementLimit = 50, int timeLimit=-1);

private:
    Destroyer& pickDestroyer();
    Repairer& pickRepairer();
    long long score(const Solution& solution) const;

    const ConfLNS* conf_;
    std::vector<std::unique_ptr<Destroyer>> destroyers_;
    std::vector<std::unique_ptr<Repairer>> repairers_;
    long long unassignedWeight_ = 0;

};
