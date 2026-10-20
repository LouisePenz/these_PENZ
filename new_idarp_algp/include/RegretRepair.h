#pragma once

#include "Repairer.h"
#include "RequestInsertionSearch.h"

// At each step, inserts the unassigned request with the highest 2-regret
// (gap between its best and second-best insertion cost -- a request with no
// second option is treated as having the largest possible regret, since
// waiting risks losing its only option), and repeats.
class RegretRepair : public Repairer {
public:
    bool apply(Solution& solution, bool allowPartial) override;
    std::string name() const override { return "regret"; }

private:
    RequestInsertionSearch search_;
};
