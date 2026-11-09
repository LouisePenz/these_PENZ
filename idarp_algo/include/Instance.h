#pragma once

#include <string>
#include <vector>

#include "PointRegistry.h"
#include "Request.h"
#include "TravelTimeProvider.h"

// The trips a request may take: wIDARP every pattern (direct, vehicle -> PT -> vehicle, walk -> PT ->
// vehicle and its reverse), IDARP no walking (direct, vehicle -> PT -> vehicle), DARP direct only.
enum class TripModel { Widarp, Idarp, Darp };
inline const char* tripModelName(TripModel model) {
    return model == TripModel::Widarp ? "widarp" : model == TripModel::Idarp ? "idarp" : "darp";
}

// What the search minimises: the vehicles' riding time (driving between stops, waiting excluded), or the
// users' riding time (from the pick-up by a vehicle to the drop-off, or to the arrival on foot).
enum class Objective { RouteTime, RidingTime };
inline const char* objectiveName(Objective objective) {
    return objective == Objective::RouteTime ? "route_time" : "riding_time";
}

class Instance {
public:
    Instance(const std::string& instanceFile, const std::string& driveTimeFile,
             const std::string& publicTransportFile, const std::string& walkFile);

    int nbRequests() const noexcept { return static_cast<int>(requests_.size()); }
    int nbVehicles() const noexcept { return nbVehicles_; }
    void setNbVehicles(int nbVehicles) noexcept { nbVehicles_ = nbVehicles; }
    int nbStops() const noexcept { return registry_.nbStops(); }
    int capacity() const noexcept { return capacity_; }
    int horizonTime() const noexcept { return horizonTime_; }
    Objective objective() const noexcept { return objective_; }
    void setObjective(Objective objective) noexcept { objective_ = objective; }

    const Request& request(int idx) const { return requests_[idx]; }
    Request& request(int idx) { return requests_[idx]; }

    const TravelTimeProvider& travelTimes() const noexcept { return travelTimes_; }

    // Optional (-tram), before configurePublicTransport(): replaces the PT
    // matrix by the tram timetable (see TravelTimeProvider::loadTramTimetable).
    // The PT file given to the constructor may then be empty.
    void loadTramTimetable(const std::string& tramFile);

    // Must be called (in this order) before precomputeAccessOptions(), so that
    // TravelTimeProvider knows the waiting time / minimum PT ride / walk threshold.
    void configurePublicTransport(int waitingTime, int minimumTimeInTransit);
    void configureWalk(int walkThreshold);

    // Precomputes, for every request, its PT options: up to maxPtOptions stop
    // pairs for the 2-vehicle pattern (Request::ptAccess), and up to
    // maxPtOptions stops on the vehicle side of the walk pattern
    // (Request::walkAccess). Requires configurePublicTransport/configureWalk first.
    // Only the options of the patterns `model` allows (none for DARP): the others are never tried.
    void precomputeAccessOptions(int maxPtOptions, TripModel model = TripModel::Widarp);

    // True if request req can go pickup -> stopBefore -> [PT] -> stopAfter -> delivery
    // while respecting its maximum ride time.
    bool isStopPairFeasible(int req, int stopBefore, int stopAfter) const;
    // The criterion for a usable (stopBefore, stopAfter) pair: ride-time
    // feasible, and PT ride at least the minimum time in transit.
    bool isValidStopPair(int req, int stopBefore, int stopAfter) const;
    // The derived data (PT time, windows, leg maximum ride times) of request
    // req going through stopBefore -> [PT] -> stopAfter, whether or not this
    // pair is among its candidates.
    StopPair stopPair(int req, int stopBefore, int stopAfter) const;

    const std::vector<int>& relatedRequests(int req) const { return related_[req]; }
    const std::vector<int>& relatedRequestsWithTransfer(int req) const { return relatedWithTransfer_[req]; }

private:
    struct Header {
        int nbRequests;
        int nbVehicles;
        int nbStops;
        int capacity;
        int horizon;
    };
    static Header readHeader(const std::string& filename);
    Instance(Header header, const std::string& instanceFile, const std::string& driveTimeFile,
             const std::string& publicTransportFile, const std::string& walkFile);

    PointRegistry registry_;
    TravelTimeProvider travelTimes_;
    std::vector<Request> requests_;

    int nbVehicles_ = 0;
    int capacity_ = 0;
    int horizonTime_ = 0;
    Objective objective_ = Objective::RouteTime;

    std::vector<std::vector<int>> related_;
    std::vector<std::vector<int>> relatedWithTransfer_;

    void loadRequests(const std::string& instanceFile);
    void computeStopPairs(int maxPairs);
    void computeWalkData(int maxStops);
    void computeRelatedness();
    void computeRelatednessWithTransfer();
};
