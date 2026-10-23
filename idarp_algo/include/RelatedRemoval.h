#pragma once

#include "Destroyer.h"

// Related (Shaw) removal, as in Ropke & Pisinger (2006): starts from a random
// assigned request, then repeatedly picks a reference among the requests
// already selected and adds the assigned, not yet selected request at rank
// floor(y^randomness * |L|) in L, those requests ranked from most to least
// related to the reference (y uniform in [0, 1)). randomness >= 1: the larger,
// the closer to always the most related one.
//
// Two relatedness measures (Molenbruch et al. 2021, Section 4.1.1), both
// precomputed by Instance from distances and time window centers:
// - PickupAndDelivery: pickups close to each other AND deliveries close to
//   each other (Instance::relatedRequests) -- the classical related removal;
// - AnyEnds: the closest of the four pairs of ends, pickup or delivery
//   (Instance::relatedRequestsWithTransfer) -- their "separate selection":
//   with transfers, a request's pickup and delivery may be served by
//   different vehicles, so one close end is enough to combine two requests.
class RelatedRemoval : public Destroyer {
public:
    enum class Measure { PickupAndDelivery, AnyEnds };

    RelatedRemoval(Measure measure, double randomness) : measure_(measure), randomness_(randomness) {}

    void apply(Solution& solution, int nbToRemove) override;
    std::string name() const override { return measure_ == Measure::AnyEnds ? "separateSelection" : "related"; }

private:
    Measure measure_;
    double randomness_;
};
