#pragma once

#include "Destroyer.h"

class RandomRemoval : public Destroyer {
public:
    void apply(Solution& solution, int nbToRemove) override;
    std::string name() const override { return "random"; }
};
