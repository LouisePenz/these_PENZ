#pragma once

// A "point" is any location referenced in the drive-time / walk-time matrices:
// - id 0 is the vehicle depot
// - ids 1..2*nbRequests are pickup and delivery points (2 per request)
// - ids 2*nbRequests+1..2*nbRequests+nbStops are public transport stops
//
// Public transport stops additionally have a *local* index (0..nbStops-1) used
// to look up the separate, smaller PT-only travel time matrix.

enum class PointType { Depot, PickupDelivery, PtStop };

class PointRegistry {
public:
    PointRegistry(int nbRequests, int nbStops);

    int nbRequests() const noexcept { return nbRequests_; }
    int nbStops() const noexcept { return nbStops_; }
    int nbVertices() const noexcept { return 1 + 2 * nbRequests_ + nbStops_; }

    PointType typeOf(int pointId) const noexcept;

    // Global point id -> local index in the PT-only matrix (valid only for PtStop points).
    int toPtLocalIndex(int pointId) const noexcept;
    // Local index in the PT-only matrix -> global point id.
    int fromPtLocalIndex(int ptLocalIndex) const noexcept;

    static constexpr int depot = 0;

private:
    int nbRequests_;
    int nbStops_;
};
