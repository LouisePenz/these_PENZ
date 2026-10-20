#pragma once

#include <string>
#include <vector>

#include "PointRegistry.h"
#include "Request.h"
#include "TravelTimeProvider.h"

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

    const Request& request(int idx) const { return requests_[idx]; }
    Request& request(int idx) { return requests_[idx]; }
    const std::vector<Request>& requests() const noexcept { return requests_; }

    const PointRegistry& pointRegistry() const noexcept { return registry_; }
    const TravelTimeProvider& travelTimes() const noexcept { return travelTimes_; }

    // Must be called (in this order) before precomputeAccessOptions(), so that
    // TravelTimeProvider knows the waiting time / minimum PT ride / walk threshold.
    void configurePublicTransport(int waitingTime, int minimumTimeInTransit);
    void configureWalk(int walkThreshold);

    // Precomputes, for every request, its PT options: up to maxPtOptions stop
    // pairs for the 2-vehicle pattern (Request::ptAccess), and up to
    // maxPtOptions stops on the vehicle side of the walk pattern
    // (Request::walkAccess). Requires configurePublicTransport/configureWalk first.
    void precomputeAccessOptions(int maxPtOptions);

    // True if request req can go pickup -> stopBefore -> [PT] -> stopAfter -> delivery
    // while respecting its maximum ride time.
    bool isStopPairFeasible(int req, int stopBefore, int stopAfter) const;
    // The criterion for a usable (stopBefore, stopAfter) pair: ride-time
    // feasible, PT ride at least the minimum time in transit, and access legs
    // shorter than the direct trip.
    bool isValidStopPair(int req, int stopBefore, int stopAfter) const;

    // In case of a transfer with a walking leg, the maximum ride time allowed
    // for the vehicle leg opposite to the walk, when the vehicle serves PT stop
    // stopId (one of walkAccess().complementaryStops()).
    int maxRideTimeLegWithWalk(int req, int stopId) const;

    const std::vector<int>& relatedRequests(int req) const { return related_[req]; }
    const std::vector<int>& relatedRequestsWithTransfer(int req) const { return relatedWithTransfer_[req]; }

    // Prints requests and general data. Deliberately omits the raw matrices.
    void print() const;

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

    std::vector<std::vector<int>> related_;
    std::vector<std::vector<int>> relatedWithTransfer_;

    void loadRequests(const std::string& instanceFile);
    void computeStopPairs(int maxPairs);
    void computeWalkData(int maxStops);
    void computeRelatedness();
    void computeRelatednessWithTransfer();
};
