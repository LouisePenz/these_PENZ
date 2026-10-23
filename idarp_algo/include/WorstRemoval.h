#pragma once

#include "Destroyer.h"

// Worst removal (Ropke & Pisinger 2006): removes, one at a time, requests
// whose removal saves the most driving cost. Randomized: the assigned
// requests are ranked by decreasing saving and the one at rank
// floor(y^randomness * n) is removed, y uniform in [0, 1). randomness >= 1:
// 1 is a uniform choice, the larger the closer to always the worst. Savings
// are recomputed after each removal (it changes those on the routes it touched).
class WorstRemoval : public Destroyer {
public:
    explicit WorstRemoval(double randomness) : randomness_(randomness) {}

    void apply(Solution& solution, int nbToRemove) override;
    std::string name() const override { return "worst"; }

private:
    double randomness_;
};
