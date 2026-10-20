#include "Instance.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <tuple>

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

void Instance::precomputeAccessOptions(int maxPtOptions) {
    computeStopPairs(maxPtOptions);
    computeWalkData(maxPtOptions);
}

bool Instance::isStopPairFeasible(int req, int stopBefore, int stopAfter) const {
    const Request& r = requests_[req];
    return travelTimes_.driveTime(r.pickupId(), stopBefore) +
               travelTimes_.publicTransportTime(stopBefore, stopAfter) +
               travelTimes_.driveTime(stopAfter, r.deliveryId()) + 3 * r.loadingTime() <=
           r.maxRideTime();
}

bool Instance::isValidStopPair(int req, int stopBefore, int stopAfter) const {
    return isStopPairFeasible(req, stopBefore, stopAfter) &&
           travelTimes_.publicTransportTime(stopBefore, stopAfter) >= travelTimes_.minimumTimeInTransit();
}

void Instance::computeStopPairs(int maxPairs) {
    int firstStop = 1 + 2 * registry_.nbRequests();
    int lastStop = firstStop + registry_.nbStops();

    for (int req = 0; req < nbRequests(); ++req) {
        Request& r = requests_[req];
        int L = r.loadingTime();
        int ltwP = r.pickupWindow().lowerBound;
        int utwD = r.deliveryWindow().upperBound;

        // Every valid pair, with its derived data. Times are service start
        // times; the request spends L at pickup, stopBefore (alighting) and
        // stopAfter (boarding) within its ride time.
        struct Ranked {
            int accessCost;
            StopPair pair;
        };
        std::vector<Ranked> valid;
        for (int sb = firstStop; sb < lastStop; ++sb) {
            for (int sa = firstStop; sa < lastStop; ++sa) {
                if (!isValidStopPair(req, sb, sa)) continue;
                int toStop = travelTimes_.driveTime(r.pickupId(), sb);
                int fromStop = travelTimes_.driveTime(sa, r.deliveryId());
                StopPair p;
                p.stopBefore = sb;
                p.stopAfter = sa;
                p.ptTravelTime = travelTimes_.publicTransportTime(sb, sa);
                p.windowBefore = {ltwP + L + toStop, utwD - L - fromStop - L - p.ptTravelTime};
                p.windowAfter = {ltwP + L + toStop + L + p.ptTravelTime, utwD - L - fromStop};
                p.maxRideTimeFirstLeg = r.maxRideTime() - 2 * L - p.ptTravelTime - fromStop;
                p.maxRideTimeLastLeg = r.maxRideTime() - 2 * L - toStop - p.ptTravelTime;
                if (p.windowBefore.lowerBound > p.windowBefore.upperBound) continue;  // no time for the whole trip
                valid.push_back({toStop + travelTimes_.driveTime(r.deliveryId(), sa), p});
            }
        }

        // Keep the maxPairs cheapest access legs (ties: stop ids, for reproducibility).
        std::sort(valid.begin(), valid.end(), [](const Ranked& a, const Ranked& b) {
            return std::tie(a.accessCost, a.pair.stopBefore, a.pair.stopAfter) <
                   std::tie(b.accessCost, b.pair.stopBefore, b.pair.stopAfter);
        });
        std::vector<StopPair> kept;
        for (size_t i = 0; i < valid.size() && static_cast<int>(kept.size()) < maxPairs; ++i) kept.push_back(valid[i].pair);
        r.ptAccess().setPairs(std::move(kept));
    }
}

void Instance::computeWalkData(int maxStops) {
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
            // Keep (up to) the maxStops closest stops whose derived window isn't empty.
            std::vector<int> selected;
            TimeWindow aggregate{horizonTime_, 0};
            for (size_t i = 0; i < stopsAfterWalk.size() && static_cast<int>(selected.size()) < maxStops; ++i) {
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
            // Keep (up to) the maxStops closest stops whose derived window isn't empty.
            std::vector<int> selected;
            TimeWindow aggregate{horizonTime_, 0};
            for (size_t i = 0; i < stopsBeforeWalk.size() && static_cast<int>(selected.size()) < maxStops; ++i) {
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
