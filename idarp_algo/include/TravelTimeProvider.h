#pragma once

#include <string>
#include <utility>
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
    // Tram timetable (-tram), in place of the PT matrix: one line per run,
    // "label nbStops stop0 time0 stop1 time1 ..." (local stop indices, times
    // in the instance's time unit), after a first line holding the number of
    // runs. Once loaded, publicTransportTime() is the shortest in-vehicle time
    // of a DIRECT run between the two stops (one tram, no waiting): a lower
    // bound for the O(1) tests, the exact STP using the runs themselves
    // (earliestArrival / latestDeparture).
    void loadTramTimetable(const std::string& filename);
    bool hasTimetable() const noexcept { return hasTimetable_; }

    static constexpr int kNoTram = 1000000;
    // Over the direct runs from stop a to stop b (global point ids) leaving a
    // no later than depCap and reaching b no earlier than arrFloor:
    // the earliest arrival at b of one leaving a at t or later (kNoTram if none),
    int earliestArrival(int a, int b, int t, int depCap = kNoTram, int arrFloor = -kNoTram) const;
    // the latest departure from a of one reaching b at t or earlier (-kNoTram if none).
    int latestDeparture(int a, int b, int t, int depCap = kNoTram, int arrFloor = -kNoTram) const;
    // The same runs, whole: (departure from a, arrival at b), {kNoTram, kNoTram} if none.
    std::pair<int, int> earliestRun(int a, int b, int t, int depCap = kNoTram, int arrFloor = -kNoTram) const;
    std::pair<int, int> latestRun(int a, int b, int t, int depCap = kNoTram, int arrFloor = -kNoTram) const;

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

    // Direct runs between two stops, Pareto-optimal (no other run leaves
    // later AND arrives earlier): departures and arrivals both increasing.
    // Dropping the others is exact as long as trams never overtake each
    // other (the assumption of Molenbruch et al.), even with depCap/arrFloor.
    struct DirectRuns {
        std::vector<int> dep;
        std::vector<int> arr;
    };
    std::vector<DirectRuns> directRuns_;  // nbStops_ x nbStops_, by local indices
    bool hasTimetable_ = false;
    const DirectRuns& directRuns(int a, int b) const {
        return directRuns_[static_cast<size_t>(a - firstPtStop_) * nbStops_ + (b - firstPtStop_)];
    }

    static void loadMatrix(int rows, int cols, std::vector<int>& matrix, const std::string& filename);
};
