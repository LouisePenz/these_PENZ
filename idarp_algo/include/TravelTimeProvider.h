#pragma once

#include <string>
#include <vector>

#include "PointRegistry.h"

// The single point of access to travel-time data: driving (all points), walking
// (all points), and public transport (PT stops only, already includes the
// average waiting time). Nobody else should touch the raw matrices directly.
class TravelTimeProvider {
public:
    explicit TravelTimeProvider(const PointRegistry& registry);

    void loadDriveTime(const std::string& filename);
    void loadWalkTime(const std::string& filename);
    void loadPublicTransportTime(const std::string& filename);

    void setWaitingTime(int waitingTime) noexcept { waitingTime_ = waitingTime; }
    void setMinimumTimeInTransit(int minimumTime) noexcept { minimumTimeInTransit_ = minimumTime; }
    void setWalkThreshold(int threshold) noexcept { walkThreshold_ = threshold; }

    // Defined here so that they get inlined: called hundreds of millions of
    // times by the insertion search.
    int driveTime(int i, int j) const noexcept { return driveTime_[static_cast<size_t>(i) * nbVertices_ + j]; }
    int walkTime(int i, int j) const noexcept { return walkTime_[static_cast<size_t>(i) * nbVertices_ + j]; }
    // Travel time in public transport between two PT stops (global point ids).
    // Already includes the average waiting time.
    int publicTransportTime(int stopBefore, int stopAfter) const noexcept {
        size_t sb = static_cast<size_t>(stopBefore - firstPtStop_);
        return publicTransportTime_[sb * nbStops_ + (stopAfter - firstPtStop_)] + waitingTime_;
    }

    int waitingTime() const noexcept { return waitingTime_; }
    int minimumTimeInTransit() const noexcept { return minimumTimeInTransit_; }
    int walkThreshold() const noexcept { return walkThreshold_; }

private:
    const PointRegistry& registry_;
    // Copied from registry_ at construction (fixed afterwards): no indirection per lookup.
    int nbVertices_;
    int nbStops_;
    int firstPtStop_;  // global point id of the PT stop with local index 0
    std::vector<int> driveTime_;
    std::vector<int> walkTime_;
    std::vector<int> publicTransportTime_;

    int waitingTime_ = 0;
    int minimumTimeInTransit_ = 0;
    int walkThreshold_ = 0;

    static void loadMatrix(int rows, int cols, std::vector<int>& matrix, const std::string& filename);
};
