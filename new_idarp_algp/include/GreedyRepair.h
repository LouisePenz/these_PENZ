#pragma once

#include "Repairer.h"
#include "RequestInsertionSearch.h"

// At each step, inserts the single cheapest feasible (request, position)
// among every currently-unassigned request, and repeats.
class GreedyRepair : public Repairer {
public:
    bool apply(Solution& solution, bool allowPartial) override;
    std::string name() const override { return "greedy"; }

private:
    RequestInsertionSearch search_;
};
