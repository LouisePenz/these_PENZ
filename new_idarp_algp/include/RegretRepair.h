#pragma once

#include "Repairer.h"
#include "RequestInsertionSearch.h"

// At each step, inserts the unassigned request with the highest 2-regret, and
// repeats. As in Ropke & Pisinger (2006), the regret is the gap between the
// cost of its best insertion and of its best insertion on OTHER routes -- the
// routes of a 2-vehicle insertion being the pair (first leg's route, second
// leg's route). A request with no option on other routes has the largest
// possible regret (waiting risks losing its only routes). Ties are broken by
// the cheapest best insertion.
class RegretRepair : public Repairer {
public:
    bool apply(Solution& solution, bool allowPartial) override;
    std::string name() const override { return "regret"; }

private:
    RequestInsertionSearch search_;
};
