#pragma once

#include <unordered_map>
#include <vector>

struct TimeWindow {
    int lowerBound = 0;
    int upperBound = 0;
};

// One way to use PT between two vehicles: pickup -> (vehicle A) -> stopBefore
// -> (PT) -> stopAfter -> (vehicle B) -> delivery. Feasibility depends on the
// PAIR (PT time, ride time), so everything derived is stored per pair.
struct StopPair {
    int stopBefore = -1;
    int stopAfter = -1;
    int ptTravelTime = 0;     // publicTransportTime(stopBefore, stopAfter), average waiting included
    TimeWindow windowBefore;  // service at stopBefore
    TimeWindow windowAfter;   // service at stopAfter
    int maxRideTimeFirstLeg = 0;  // pickup -> stopBefore
    int maxRideTimeLastLeg = 0;   // stopAfter -> delivery
};

// Candidate stop pairs for the "vehicle -> PT -> vehicle" pattern: the best
// valid pairs (Instance::isValidStopPair), cheapest access legs first,
// precomputed once -- see Instance::computeStopPairs.
class PtLegAccess {
public:
    bool available() const noexcept { return !pairs_.empty(); }
    const std::vector<StopPair>& pairs() const noexcept { return pairs_; }
    void setPairs(std::vector<StopPair> pairs) { pairs_ = std::move(pairs); }

private:
    std::vector<StopPair> pairs_;
};

enum class WalkSide { None, AtPickup, AtDelivery };

// Candidate PT stops for the "walk -> PT -> vehicle" / "vehicle -> PT -> walk" patterns:
// one nearest stop reachable on foot, and a few candidate stops on the other side.
//
// Since the walk and the PT ride have constant durations, the walk + PT part
// is folded into derived data for each candidate stop: the time window of the
// vehicle's stop there, and the maximum ride time of the vehicle leg alone
// (pickup -> stop, or stop -> delivery). Any schedule respecting both is
// feasible for the whole trip -- see Instance::computeWalkData.
class WalkAccess {
public:
    bool available() const noexcept { return side_ != WalkSide::None; }
    WalkSide side() const noexcept { return side_; }

    int nearestStop() const noexcept { return nearestStop_; }
    int walkTime() const noexcept { return walkTime_; }
    const std::vector<int>& complementaryStops() const noexcept { return complementaryStops_; }
    TimeWindow windowForStop(int stopId) const { return windows_.at(stopId); }
    int maxRideTimeForStop(int stopId) const { return maxRideTimes_.at(stopId); }

    void setSide(WalkSide side) noexcept { side_ = side; }
    void setNearestStop(int stop) noexcept { nearestStop_ = stop; }
    void setWalkTime(int t) noexcept { walkTime_ = t; }
    void setComplementaryStops(std::vector<int> stops) { complementaryStops_ = std::move(stops); }
    void setWindowForStop(int stopId, TimeWindow w) { windows_[stopId] = w; }
    void setMaxRideTimeForStop(int stopId, int maxRideTime) { maxRideTimes_[stopId] = maxRideTime; }

private:
    WalkSide side_ = WalkSide::None;
    int nearestStop_ = -1;
    int walkTime_ = 0;
    std::vector<int> complementaryStops_;
    std::unordered_map<int, TimeWindow> windows_;
    std::unordered_map<int, int> maxRideTimes_;
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
