#pragma once

#include <string>
#include <unordered_map>
#include <vector>

struct TimeWindow {
    int lowerBound = 0;
    int upperBound = 0;
};

// Candidate PT stops for the "vehicle -> PT -> vehicle" pattern: up to a few
// best stops before the PT leg, and a few best stops after, precomputed once.
class PtLegAccess {
public:
    bool available() const noexcept { return !stopsBefore_.empty(); }

    const std::vector<int>& stopsBefore() const noexcept { return stopsBefore_; }
    const std::vector<int>& stopsAfter() const noexcept { return stopsAfter_; }

    TimeWindow windowForStopBefore(int stopId) const { return windowBefore_.at(stopId); }
    TimeWindow windowForStopAfter(int stopId) const { return windowAfter_.at(stopId); }

    // Widest time window covering every candidate stop before/after (union of the per-stop windows).
    TimeWindow aggregateWindowBefore() const noexcept { return aggregateBefore_; }
    TimeWindow aggregateWindowAfter() const noexcept { return aggregateAfter_; }

    void setStopsBefore(std::vector<int> stops) { stopsBefore_ = std::move(stops); }
    void setStopsAfter(std::vector<int> stops) { stopsAfter_ = std::move(stops); }
    void setWindowForStopBefore(int stopId, TimeWindow w) { windowBefore_[stopId] = w; }
    void setWindowForStopAfter(int stopId, TimeWindow w) { windowAfter_[stopId] = w; }
    void setAggregateWindowBefore(TimeWindow w) noexcept { aggregateBefore_ = w; }
    void setAggregateWindowAfter(TimeWindow w) noexcept { aggregateAfter_ = w; }

private:
    std::vector<int> stopsBefore_;
    std::vector<int> stopsAfter_;
    std::unordered_map<int, TimeWindow> windowBefore_;
    std::unordered_map<int, TimeWindow> windowAfter_;
    TimeWindow aggregateBefore_;
    TimeWindow aggregateAfter_;
};

enum class WalkSide { None, AtPickup, AtDelivery };

// Candidate PT stops for the "walk -> PT -> vehicle" / "vehicle -> PT -> walk" patterns:
// one nearest stop reachable on foot, and a few candidate stops on the other side.
class WalkAccess {
public:
    bool available() const noexcept { return side_ != WalkSide::None; }
    WalkSide side() const noexcept { return side_; }

    int nearestStop() const noexcept { return nearestStop_; }
    int walkTime() const noexcept { return walkTime_; }
    const std::vector<int>& complementaryStops() const noexcept { return complementaryStops_; }
    TimeWindow windowForStop(int stopId) const { return windows_.at(stopId); }
    TimeWindow aggregateWindow() const noexcept { return aggregateWindow_; }

    void setSide(WalkSide side) noexcept { side_ = side; }
    void setNearestStop(int stop) noexcept { nearestStop_ = stop; }
    void setWalkTime(int t) noexcept { walkTime_ = t; }
    void setComplementaryStops(std::vector<int> stops) { complementaryStops_ = std::move(stops); }
    void setWindowForStop(int stopId, TimeWindow w) { windows_[stopId] = w; }
    void setAggregateWindow(TimeWindow w) noexcept { aggregateWindow_ = w; }

private:
    WalkSide side_ = WalkSide::None;
    int nearestStop_ = -1;
    int walkTime_ = 0;
    std::vector<int> complementaryStops_;
    std::unordered_map<int, TimeWindow> windows_;
    TimeWindow aggregateWindow_;
};

class Request {
public:
    Request(int pickupId, TimeWindow pickupWindow, int deliveryId, TimeWindow deliveryWindow,
            int quantity, int maxRideTime, int loadingTime);

    int pickupId() const noexcept { return pickupId_; }
    TimeWindow pickupWindow() const noexcept { return pickupWindow_; }
    int deliveryId() const noexcept { return deliveryId_; }
    TimeWindow deliveryWindow() const noexcept { return deliveryWindow_; }
    int quantity() const noexcept { return quantity_; }
    int maxRideTime() const noexcept { return maxRideTime_; }
    int loadingTime() const noexcept { return loadingTime_; }

    PtLegAccess& ptAccess() noexcept { return ptAccess_; }
    const PtLegAccess& ptAccess() const noexcept { return ptAccess_; }
    WalkAccess& walkAccess() noexcept { return walkAccess_; }
    const WalkAccess& walkAccess() const noexcept { return walkAccess_; }

    std::string toString() const;
    void print() const;

private:
    int pickupId_;
    TimeWindow pickupWindow_;
    int deliveryId_;
    TimeWindow deliveryWindow_;
    int quantity_;
    int maxRideTime_;
    int loadingTime_;

    PtLegAccess ptAccess_;
    WalkAccess walkAccess_;
};
