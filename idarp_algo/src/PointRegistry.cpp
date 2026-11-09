#include "PointRegistry.h"

PointRegistry::PointRegistry(int nbRequests, int nbStops)
    : nbRequests_(nbRequests), nbStops_(nbStops) {}

int PointRegistry::fromPtLocalIndex(int ptLocalIndex) const noexcept {
    return ptLocalIndex + (1 + 2 * nbRequests_);
}
