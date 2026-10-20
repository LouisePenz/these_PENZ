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

    int driveTime(int i, int j) const noexcept;
    int walkTime(int i, int j) const noexcept;
    // Travel time in public transport between two PT stops (global point ids).
    // Already includes the average waiting time.
    int publicTransportTime(int stopBefore, int stopAfter) const noexcept;

    int waitingTime() const noexcept { return waitingTime_; }
    int minimumTimeInTransit() const noexcept { return minimumTimeInTransit_; }
    int walkThreshold() const noexcept { return walkThreshold_; }

private:
    const PointRegistry& registry_;
    std::vector<int> driveTime_;
    std::vector<int> walkTime_;
    std::vector<int> publicTransportTime_;

    int waitingTime_ = 0;
    int minimumTimeInTransit_ = 0;
    int walkThreshold_ = 0;

    static void loadMatrix(int rows, int cols, std::vector<int>& matrix, const std::string& filename);
};
