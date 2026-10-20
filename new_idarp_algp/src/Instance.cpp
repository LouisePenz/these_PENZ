#include "Instance.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

Instance::Header Instance::readHeader(const std::string& filename) {
    std::ifstream file(filename, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "The file of your problem cannot be open: " << filename << std::endl;
        exit(1);
    }
    Header h;
    file >> h.nbRequests >> h.nbVehicles >> h.nbStops >> h.capacity >> h.horizon;
    return h;
}

Instance::Instance(const std::string& instanceFile, const std::string& driveTimeFile,
                    const std::string& publicTransportFile, const std::string& walkFile)
    : Instance(readHeader(instanceFile), instanceFile, driveTimeFile, publicTransportFile, walkFile) {}

Instance::Instance(Header h, const std::string& instanceFile, const std::string& driveTimeFile,
                    const std::string& publicTransportFile, const std::string& walkFile)
    : registry_(h.nbRequests, h.nbStops),
      travelTimes_(registry_),
      nbVehicles_(h.nbVehicles),
      capacity_(h.capacity),
      horizonTime_(h.horizon) {
    loadRequests(instanceFile);
    travelTimes_.loadDriveTime(driveTimeFile);
    travelTimes_.loadPublicTransportTime(publicTransportFile);
    travelTimes_.loadWalkTime(walkFile);

    computeRelatedness();
    computeRelatednessWithTransfer();
}

void Instance::loadRequests(const std::string& instanceFile) {
    std::ifstream file(instanceFile, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "The file of your problem cannot be open: " << instanceFile << std::endl;
        exit(1);
    }
    int nbRequests, nbVehicles, nbStops, capacity, horizon;
    file >> nbRequests >> nbVehicles >> nbStops >> capacity >> horizon;

    requests_.reserve(nbRequests);
    int idP, idD, quantity, ltwP, utwP, ltwD, utwD, maxRideTime, loadingTime;
    for (int r = 0; r < nbRequests; ++r) {
        file >> idP >> ltwP >> utwP >> idD >> ltwD >> utwD >> maxRideTime >> quantity >> loadingTime;
        requests_.emplace_back(idP, TimeWindow{ltwP, utwP}, idD, TimeWindow{ltwD, utwD}, quantity,
                                maxRideTime, loadingTime);
    }
}

void Instance::configurePublicTransport(int waitingTime, int minimumTimeInTransit) {
    travelTimes_.setWaitingTime(waitingTime);
    travelTimes_.setMinimumTimeInTransit(minimumTimeInTransit);
}

void Instance::configureWalk(int walkThreshold) { travelTimes_.setWalkThreshold(walkThreshold); }

void Instance::precomputeAccessOptions() {
    computeStopsBeforeAndAfter();
    computeWalkData();
}

bool Instance::isStopPairFeasible(int req, int stopBefore, int stopAfter) const {
    const Request& r = requests_[req];
    return travelTimes_.driveTime(r.pickupId(), stopBefore) +
               travelTimes_.publicTransportTime(stopBefore, stopAfter) +
               travelTimes_.driveTime(stopAfter, r.deliveryId()) + 3 * r.loadingTime() <=
           r.maxRideTime();
}

bool Instance::isValidStopPair(int req, int stopBefore, int stopAfter) const {
    const Request& r = requests_[req];
    int accessCost = travelTimes_.driveTime(r.pickupId(), stopBefore) + travelTimes_.driveTime(r.deliveryId(), stopAfter);
    return isStopPairFeasible(req, stopBefore, stopAfter) &&
           travelTimes_.publicTransportTime(stopBefore, stopAfter) >= travelTimes_.minimumTimeInTransit() &&
           accessCost < travelTimes_.driveTime(r.pickupId(), r.deliveryId());
}

TimeWindow Instance::stopAfterWindow(int req, int stopBefore, int stopAfter) const {
    const Request& r = requests_[req];
    TimeWindow w;
    w.lowerBound = r.pickupWindow().lowerBound + r.loadingTime() + travelTimes_.driveTime(r.pickupId(), stopBefore) +
                   r.loadingTime() + travelTimes_.publicTransportTime(stopBefore, stopAfter);
    w.upperBound = r.deliveryWindow().upperBound - r.loadingTime() - travelTimes_.driveTime(stopAfter, r.deliveryId());
    return w;
}

void Instance::computeStopsBeforeAndAfter() {
    int firstStop = 1 + 2 * registry_.nbRequests();
    int lastStop = firstStop + registry_.nbStops();

    for (int req = 0; req < nbRequests(); ++req) {
        Request& r = requests_[req];
        std::vector<int> stopsBefore, stopsAfter;
        // (stopBefore, stopAfter) -> cost of that travel, kept only if it beats the direct trip.
        std::vector<std::pair<std::pair<int, int>, int>> possibleStops;

        for (int sb = firstStop; sb < lastStop; ++sb) {
            for (int sa = firstStop; sa < lastStop; ++sa) {
                if (isValidStopPair(req, sb, sa)) {
                    int cost = travelTimes_.driveTime(r.pickupId(), sb) + travelTimes_.driveTime(r.deliveryId(), sa);
                    possibleStops.push_back({{sb, sa}, cost});
                }
            }
        }
        std::sort(possibleStops.begin(), possibleStops.end(),
                  [](const auto& a, const auto& b) { return a.second < b.second; });

        // Keep (up to) the 3 best stops before and the 3 best stops after, picked
        // greedily from the same cost-sorted pool (not two independent top-3 searches).
        size_t x = 0;
        while (x < possibleStops.size() && (stopsBefore.size() < 3 && stopsAfter.size() < 3)) {
            int sb = possibleStops[x].first.first;
            int sa = possibleStops[x].first.second;
            if (std::find(stopsBefore.begin(), stopsBefore.end(), sb) == stopsBefore.end())
                stopsBefore.push_back(sb);
            if (std::find(stopsAfter.begin(), stopsAfter.end(), sa) == stopsAfter.end())
                stopsAfter.push_back(sa);
            ++x;
        }
        r.ptAccess().setStopsBefore(stopsBefore);
        r.ptAccess().setStopsAfter(stopsAfter);

        if (stopsBefore.empty()) continue;

        int ltwP = r.pickupWindow().lowerBound;
        int utwD = r.deliveryWindow().upperBound;
        int loadingTime = r.loadingTime();

        TimeWindow aggregateBefore{horizonTime_, 0};
        TimeWindow aggregateAfter{horizonTime_, 0};

        for (int sb : stopsBefore) {
            TimeWindow w;
            w.lowerBound = ltwP + loadingTime + travelTimes_.driveTime(r.pickupId(), sb);
            w.upperBound = 0;
            for (int sa : stopsAfter) {
                if (isValidStopPair(req, sb, sa)) {
                    int utw = utwD - loadingTime - travelTimes_.driveTime(sa, r.deliveryId()) - loadingTime -
                              travelTimes_.publicTransportTime(sb, sa);
                    if (utw > w.upperBound) w.upperBound = utw;
                }
            }
            r.ptAccess().setWindowForStopBefore(sb, w);
            if (w.upperBound > aggregateBefore.upperBound) aggregateBefore.upperBound = w.upperBound;
            if (w.lowerBound < aggregateBefore.lowerBound) aggregateBefore.lowerBound = w.lowerBound;
        }

        for (int sa : stopsAfter) {
            TimeWindow w;
            w.upperBound = utwD - loadingTime - travelTimes_.driveTime(sa, r.deliveryId());
            w.lowerBound = horizonTime_;
            for (int sb : stopsBefore) {
                if (isValidStopPair(req, sb, sa)) {
                    int ltw = ltwP + loadingTime + travelTimes_.driveTime(r.pickupId(), sb) + loadingTime +
                              travelTimes_.publicTransportTime(sb, sa);
                    if (ltw < w.lowerBound) w.lowerBound = ltw;
                }
            }
            r.ptAccess().setWindowForStopAfter(sa, w);
            if (w.upperBound > aggregateAfter.upperBound) aggregateAfter.upperBound = w.upperBound;
            if (w.lowerBound < aggregateAfter.lowerBound) aggregateAfter.lowerBound = w.lowerBound;
        }

        r.ptAccess().setAggregateWindowBefore(aggregateBefore);
        r.ptAccess().setAggregateWindowAfter(aggregateAfter);
    }
}

void Instance::computeWalkData() {
    int firstStop = 1 + 2 * registry_.nbRequests();
    int lastStop = firstStop + registry_.nbStops();
    int threshold = travelTimes_.walkThreshold();

    for (int req = 0; req < nbRequests(); ++req) {
        Request& r = requests_[req];

        int nearestStopForPickup = -1, timeFromPickupToNearestStop = horizonTime_;
        int nearestStopForDelivery = -1, timeFromDeliveryToNearestStop = horizonTime_;
        for (int s = firstStop; s < lastStop; ++s) {
            if (travelTimes_.walkTime(r.pickupId(), s) < timeFromPickupToNearestStop) {
                nearestStopForPickup = s;
                timeFromPickupToNearestStop = travelTimes_.walkTime(r.pickupId(), s);
            }
            if (travelTimes_.walkTime(r.deliveryId(), s) < timeFromDeliveryToNearestStop) {
                nearestStopForDelivery = s;
                timeFromDeliveryToNearestStop = travelTimes_.walkTime(r.deliveryId(), s);
            }
        }

        if (timeFromPickupToNearestStop <= threshold && timeFromDeliveryToNearestStop <= threshold) {
            std::cerr << "For request " << req << " both pickup and delivery are close to public transport, "
                      << "we stop here\n";
            std::cerr << "Stop " << nearestStopForPickup << " at " << timeFromPickupToNearestStop
                      << " and stop " << nearestStopForDelivery << " at " << timeFromDeliveryToNearestStop
                      << " with threshold at " << threshold << "\n";
            exit(1);
        } else if (timeFromPickupToNearestStop <= threshold) {
            // pickup --(walk)--> nearestStopForPickup --(PT)--> s --(vehicle)--> delivery
            // The request leaves its pickup point at t(s) - walk - pt (no waiting
            // at s), which must lie in the pickup window; the vehicle leg s ->
            // delivery gets what remains of the maximum ride time.
            int walk = timeFromPickupToNearestStop;
            std::vector<std::pair<int, int>> stopsAfterWalk;  // (candidate stop, drive time from it to delivery)
            for (int s = firstStop; s < lastStop; ++s) {
                if (walk + travelTimes_.publicTransportTime(nearestStopForPickup, s) + r.loadingTime() +
                        travelTimes_.driveTime(s, r.deliveryId()) <=
                    r.maxRideTime()) {
                    stopsAfterWalk.push_back({s, travelTimes_.driveTime(s, r.deliveryId())});
                }
            }
            std::sort(stopsAfterWalk.begin(), stopsAfterWalk.end(),
                      [](const auto& a, const auto& b) { return a.second < b.second; });
            // Keep (up to) the 3 closest stops whose derived window isn't empty.
            std::vector<int> selected;
            TimeWindow aggregate{horizonTime_, 0};
            for (size_t i = 0; i < stopsAfterWalk.size() && selected.size() < 3; ++i) {
                int s = stopsAfterWalk[i].first;
                int pt = travelTimes_.publicTransportTime(nearestStopForPickup, s);
                TimeWindow w;
                w.lowerBound = r.pickupWindow().lowerBound + walk + pt;
                w.upperBound = std::min(r.pickupWindow().upperBound + walk + pt,
                                        r.deliveryWindow().upperBound - r.loadingTime() -
                                            travelTimes_.driveTime(s, r.deliveryId()));
                if (w.lowerBound > w.upperBound) continue;
                selected.push_back(s);
                r.walkAccess().setWindowForStop(s, w);
                r.walkAccess().setMaxRideTimeForStop(s, r.maxRideTime() - walk - pt);
                if (w.lowerBound < aggregate.lowerBound) aggregate.lowerBound = w.lowerBound;
                if (w.upperBound > aggregate.upperBound) aggregate.upperBound = w.upperBound;
            }
            if (selected.empty()) {
                r.walkAccess().setSide(WalkSide::None);
                continue;
            }
            r.walkAccess().setNearestStop(nearestStopForPickup);
            r.walkAccess().setWalkTime(walk);
            r.walkAccess().setSide(WalkSide::AtPickup);
            r.walkAccess().setAggregateWindow(aggregate);
            r.walkAccess().setComplementaryStops(selected);
        } else if (timeFromDeliveryToNearestStop <= threshold) {
            // pickup --(vehicle)--> s --(PT)--> nearestStopForDelivery --(walk)--> delivery
            // The request reaches its delivery point at t(s) + loadingTime + pt +
            // walk, which must lie in the delivery window; the vehicle leg
            // pickup -> s gets what remains of the maximum ride time.
            int walk = timeFromDeliveryToNearestStop;
            std::vector<std::pair<int, int>> stopsBeforeWalk;  // (candidate stop, drive time from pickup to it)
            for (int s = firstStop; s < lastStop; ++s) {
                if (r.loadingTime() + travelTimes_.driveTime(r.pickupId(), s) + r.loadingTime() +
                        travelTimes_.publicTransportTime(s, nearestStopForDelivery) + walk <=
                    r.maxRideTime()) {
                    stopsBeforeWalk.push_back({s, travelTimes_.driveTime(r.pickupId(), s)});
                }
            }
            std::sort(stopsBeforeWalk.begin(), stopsBeforeWalk.end(),
                      [](const auto& a, const auto& b) { return a.second < b.second; });
            // Keep (up to) the 3 closest stops whose derived window isn't empty.
            std::vector<int> selected;
            TimeWindow aggregate{horizonTime_, 0};
            for (size_t i = 0; i < stopsBeforeWalk.size() && selected.size() < 3; ++i) {
                int s = stopsBeforeWalk[i].first;
                int pt = travelTimes_.publicTransportTime(s, nearestStopForDelivery);
                TimeWindow w;
                w.lowerBound = std::max(r.pickupWindow().lowerBound + r.loadingTime() +
                                            travelTimes_.driveTime(r.pickupId(), s),
                                        r.deliveryWindow().lowerBound - r.loadingTime() - pt - walk);
                w.upperBound = r.deliveryWindow().upperBound - r.loadingTime() - pt - walk;
                if (w.lowerBound > w.upperBound) continue;
                selected.push_back(s);
                r.walkAccess().setWindowForStop(s, w);
                r.walkAccess().setMaxRideTimeForStop(s, r.maxRideTime() - r.loadingTime() - pt - walk);
                if (w.lowerBound < aggregate.lowerBound) aggregate.lowerBound = w.lowerBound;
                if (w.upperBound > aggregate.upperBound) aggregate.upperBound = w.upperBound;
            }
            if (selected.empty()) {
                r.walkAccess().setSide(WalkSide::None);
                continue;
            }
            r.walkAccess().setNearestStop(nearestStopForDelivery);
            r.walkAccess().setWalkTime(timeFromDeliveryToNearestStop);
            r.walkAccess().setSide(WalkSide::AtDelivery);
            r.walkAccess().setAggregateWindow(aggregate);
            r.walkAccess().setComplementaryStops(selected);
        } else {
            r.walkAccess().setSide(WalkSide::None);
        }
    }
}

// Both legs: the loading times at stopBefore (alighting) and stopAfter
// (boarding) are spent outside the leg being bounded, hence the 2 * loadingTime.
int Instance::maxRideTimeFirstLeg(int req) const {
    const Request& r = requests_[req];
    if (!r.ptAccess().available()) return -1;
    int mrt = r.maxRideTime() - 2 * r.loadingTime();
    int mrtLeg = 0;
    for (int sb : r.ptAccess().stopsBefore()) {
        for (int sa : r.ptAccess().stopsAfter()) {
            if (!isValidStopPair(req, sb, sa)) continue;
            int candidate = mrt - (travelTimes_.publicTransportTime(sb, sa) + travelTimes_.driveTime(sa, r.deliveryId()));
            if (candidate > mrtLeg) mrtLeg = candidate;
        }
    }
    return mrtLeg;
}

int Instance::maxRideTimeLastLeg(int req) const {
    const Request& r = requests_[req];
    if (!r.ptAccess().available()) return -1;
    int mrt = r.maxRideTime() - 2 * r.loadingTime();
    int mrtLeg = 0;
    for (int sb : r.ptAccess().stopsBefore()) {
        for (int sa : r.ptAccess().stopsAfter()) {
            if (!isValidStopPair(req, sb, sa)) continue;
            int candidate = mrt - (travelTimes_.publicTransportTime(sb, sa) + travelTimes_.driveTime(r.pickupId(), sb));
            if (candidate > mrtLeg) mrtLeg = candidate;
        }
    }
    return mrtLeg;
}

int Instance::maxRideTimeLegWithWalk(int req, int stopId) const {
    const Request& r = requests_[req];
    if (!r.walkAccess().available()) return -1;
    return r.walkAccess().maxRideTimeForStop(stopId);
}

void Instance::computeRelatedness() {
    double timeMax = 0;
    for (int r1 = 0; r1 < nbRequests(); ++r1)
        for (int r2 = 0; r2 < nbRequests(); ++r2) {
            double d = requests_[r2].deliveryWindow().upperBound - requests_[r1].pickupWindow().lowerBound;
            if (d > timeMax) timeMax = d;
        }

    double distanceMax = 0;
    for (int i = 0; i < registry_.nbVertices(); ++i)
        for (int j = 0; j < registry_.nbVertices(); ++j)
            distanceMax = std::max(distanceMax, static_cast<double>(travelTimes_.driveTime(i, j)));

    related_.clear();
    for (int r1 = 0; r1 < nbRequests(); ++r1) {
        const Request& req1 = requests_[r1];
        std::vector<std::pair<double, int>> scored;
        for (int r2 = 0; r2 < nbRequests(); ++r2) {
            if (r1 == r2) continue;
            const Request& req2 = requests_[r2];
            double alpha = travelTimes_.driveTime(req1.pickupId(), req2.pickupId()) +
                           travelTimes_.driveTime(req1.deliveryId(), req2.deliveryId());
            double midP1 = req1.pickupWindow().lowerBound + (req1.pickupWindow().upperBound - req1.pickupWindow().lowerBound) / 2.0;
            double midP2 = req2.pickupWindow().lowerBound + (req2.pickupWindow().upperBound - req2.pickupWindow().lowerBound) / 2.0;
            double midD1 = req1.deliveryWindow().lowerBound + (req1.deliveryWindow().upperBound - req1.deliveryWindow().lowerBound) / 2.0;
            double midD2 = req2.deliveryWindow().lowerBound + (req2.deliveryWindow().upperBound - req2.deliveryWindow().lowerBound) / 2.0;
            double beta = std::abs(midP1 - midP2) + std::abs(midD1 - midD2);
            scored.push_back({std::pow(alpha / distanceMax + beta / timeMax, -1), r2});
        }
        std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        std::vector<int> ordered;
        ordered.reserve(scored.size());
        for (auto& p : scored) ordered.push_back(p.second);
        related_.push_back(std::move(ordered));
    }
}

void Instance::computeRelatednessWithTransfer() {
    double timeMax = 0;
    for (int r1 = 0; r1 < nbRequests(); ++r1)
        for (int r2 = 0; r2 < nbRequests(); ++r2) {
            double d = requests_[r2].deliveryWindow().upperBound - requests_[r1].pickupWindow().lowerBound;
            if (d > timeMax) timeMax = d;
        }

    double distanceMax = 0;
    for (int i = 0; i < registry_.nbVertices(); ++i)
        for (int j = 0; j < registry_.nbVertices(); ++j)
            distanceMax = std::max(distanceMax, static_cast<double>(travelTimes_.driveTime(i, j)));

    relatedWithTransfer_.clear();
    for (int r1 = 0; r1 < nbRequests(); ++r1) {
        const Request& req1 = requests_[r1];
        std::vector<std::pair<double, int>> scored;
        for (int r2 = 0; r2 < nbRequests(); ++r2) {
            if (r1 == r2) continue;
            const Request& req2 = requests_[r2];
            double twP1 = req1.pickupWindow().lowerBound + (req1.pickupWindow().upperBound - req1.pickupWindow().lowerBound) / 2.0;
            double twD1 = req1.deliveryWindow().lowerBound + (req1.deliveryWindow().upperBound - req1.deliveryWindow().lowerBound) / 2.0;
            double twP2 = req2.pickupWindow().lowerBound + (req2.pickupWindow().upperBound - req2.pickupWindow().lowerBound) / 2.0;
            double twD2 = req2.deliveryWindow().lowerBound + (req2.deliveryWindow().upperBound - req2.deliveryWindow().lowerBound) / 2.0;

            double alpha = travelTimes_.driveTime(req1.pickupId(), req2.pickupId()) / distanceMax;
            double beta = std::abs(twP1 - twP2) / timeMax;
            double minVal = alpha + beta;

            alpha = travelTimes_.driveTime(req1.pickupId(), req2.deliveryId()) / distanceMax;
            beta = std::abs(twP1 - twD2) / timeMax;
            minVal = std::min(minVal, alpha + beta);

            alpha = travelTimes_.driveTime(req1.deliveryId(), req2.pickupId()) / distanceMax;
            beta = std::abs(twD1 - twP2) / timeMax;
            minVal = std::min(minVal, alpha + beta);

            alpha = travelTimes_.driveTime(req1.deliveryId(), req2.deliveryId()) / distanceMax;
            beta = std::abs(twD1 - twD2) / timeMax;
            minVal = std::min(minVal, alpha + beta);

            scored.push_back({std::pow(minVal, -1), r2});
        }
        std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        std::vector<int> ordered;
        ordered.reserve(scored.size());
        for (auto& p : scored) ordered.push_back(p.second);
        relatedWithTransfer_.push_back(std::move(ordered));
    }
}

void Instance::print() const {
    std::cout << "There are " << nbVehicles_ << " vehicles, capacity " << capacity_ << "\n";
    std::cout << "There are " << registry_.nbStops() << " stops.\n";
    std::cout << "Horizon time is " << horizonTime_ << "\n";
    std::cout << "The average waiting time at stops is " << travelTimes_.waitingTime() << "\n";
    std::cout << "Minimum time to pass in public transportation is " << travelTimes_.minimumTimeInTransit() << "\n";
    std::cout << "The maximum walk time is " << travelTimes_.walkThreshold() << "\n";
    std::cout << "There are " << nbRequests() << " requests.\n";
    for (int req = 0; req < nbRequests(); ++req) {
        std::cout << "Request index " << req << ":\n";
        requests_[req].print();
    }
}
