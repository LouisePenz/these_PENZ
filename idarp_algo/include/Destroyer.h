#pragma once

#include <string>

class Solution;

// A destruction operator: removes up to nbToRemove requests from `solution`
// (moving them to unassignedRequests()), each concrete strategy choosing
// which ones differently (random, related, worst, route, cluster,
// separate-selection -- see the discussion on which ones to keep).
class Destroyer {
public:
    virtual ~Destroyer() = default;

    // May remove fewer than nbToRemove if the solution doesn't have that many
    // assigned requests. Leaves the solution's timing data refreshed
    // (Solution::refreshTiming) for every route it touched (removing stops
    // can only relax constraints, so this can never make a previously
    // feasible route infeasible).
    virtual void apply(Solution& solution, int nbToRemove) = 0;

    virtual std::string name() const = 0;
};
