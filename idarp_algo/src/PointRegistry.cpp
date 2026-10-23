#include "PointRegistry.h"

PointRegistry::PointRegistry(int nbRequests, int nbStops)
    : nbRequests_(nbRequests), nbStops_(nbStops) {}

PointType PointRegistry::typeOf(int pointId) const noexcept {
    if (pointId == depot) return PointType::Depot;
    if (pointId <= 2 * nbRequests_) return PointType::PickupDelivery;
    return PointType::PtStop;
}

int PointRegistry::toPtLocalIndex(int pointId) const noexcept {
    return pointId - (1 + 2 * nbRequests_);
}

int PointRegistry::fromPtLocalIndex(int ptLocalIndex) const noexcept {
    return ptLocalIndex + (1 + 2 * nbRequests_);
}
