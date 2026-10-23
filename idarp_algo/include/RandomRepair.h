#pragma once

#include "Repairer.h"
#include "RequestInsertionSearch.h"

// Tries every unassigned request, in random order, inserting each at its
// single cheapest feasible position.
class RandomRepair : public Repairer {
public:
    bool apply(Solution& solution, bool allowPartial) override;
    std::string name() const override { return "random"; }

private:
    RequestInsertionSearch search_;
};
