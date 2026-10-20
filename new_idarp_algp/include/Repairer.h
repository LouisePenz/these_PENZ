#pragma once

#include <string>

class Solution;

// A repair operator: attempts to insert every currently-unassigned request
// back into `solution` (random, greedy, regret -- see the discussion on
// which ones to keep).
class Repairer {
public:
    virtual ~Repairer() = default;

    // allowPartial = false (the "full" LNS): stops at the first request that
    // cannot be inserted feasibly, leaves it and the rest unassigned, and
    // returns false -- the caller must discard this attempt (the solution is
    // not guaranteed usable as-is).
    //
    // allowPartial = true (PartialLNS, e.g. building an initial solution from
    // empty routes): keeps trying every remaining request even if some fail,
    // and always returns true; some requests may still be left in
    // solution.unassignedRequests() afterwards -- that's not an error here.
    virtual bool apply(Solution& solution, bool allowPartial) = 0;

    virtual std::string name() const = 0;
};
