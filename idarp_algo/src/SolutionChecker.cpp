#include "SolutionChecker.h"

#include <algorithm>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "Solution.h"

namespace {

// Collects violations by kind, so that a systematic error (e.g. the same
// wrong service time on every PT stop) shows up as one line with a count
// rather than hundreds of identical messages.
class ViolationLog {
public:
    void add(const std::string& kind, const std::string& detail) {
        auto& entry = byKind_[kind];
        ++entry.count;
        if (entry.examples.size() < kMaxExamples) entry.examples.push_back(detail);
        ++total_;
    }

    int total() const noexcept { return total_; }

    void print(std::ostream& out) const {
        for (const auto& [kind, entry] : byKind_) {
            out << "   [" << entry.count << "x] " << kind << "\n";
            for (const auto& e : entry.examples) out << "         " << e << "\n";
        }
    }

private:
    static constexpr size_t kMaxExamples = 5;
    struct Entry {
        int count = 0;
        std::vector<std::string> examples;
    };
    std::map<std::string, Entry> byKind_;
    int total_ = 0;
};

struct PlacedStop {
    StopRole role;
    int route;
    int position;
    int pointId;
    int time;
};

std::string req(int requestId) { return "req " + std::to_string(requestId); }

std::string at(const PlacedStop& s) {
    return "(route " + std::to_string(s.route) + ", pos " + std::to_string(s.position) + ", t=" +
           std::to_string(s.time) + ")";
}

bool contains(const std::vector<int>& v, int x) { return std::find(v.begin(), v.end(), x) != v.end(); }

void checkWindow(ViolationLog& log, const std::string& kind, int requestId, int time, TimeWindow w) {
    if (time < w.lowerBound || time > w.upperBound) {
        log.add(kind, req(requestId) + ": t=" + std::to_string(time) + " outside [" + std::to_string(w.lowerBound) +
                          ";" + std::to_string(w.upperBound) + "]");
    }
}

void checkRideTime(ViolationLog& log, int requestId, int rideTime, int maxRideTime) {
    if (rideTime > maxRideTime) {
        log.add("max ride time exceeded",
                req(requestId) + ": ride " + std::to_string(rideTime) + " > " + std::to_string(maxRideTime));
    }
}

// Same route, `first` strictly before `second`.
bool checkOrder(ViolationLog& log, int requestId, const PlacedStop& first, const PlacedStop& second) {
    if (first.route != second.route || first.position >= second.position) {
        log.add("stops of one vehicle leg not in order on a single route",
                req(requestId) + ": " + at(first) + " then " + at(second));
        return false;
    }
    return true;
}

}  // namespace

// Checks the schedule stored in ltwDarp of `scheduled` against the model.
static int checkSchedule(const Solution& scheduled, const std::string& label, std::ostream& out,
                         bool requireCandidatePairs) {
    const Instance& instance = scheduled.instance();
    const TravelTimeProvider& tt = instance.travelTimes();
    const int horizon = instance.horizonTime();
    ViolationLog log;

    // --- Per route: travel times, service times, depot return, capacity. ---
    std::map<int, std::vector<PlacedStop>> stopsByRequest;
    for (int r = 0; r < scheduled.nbRoutes(); ++r) {
        const Route& route = scheduled.route(r);
        int prevPoint = route.depotPointId();
        int prevTime = 0;
        int prevService = 0;
        int load = 0;

        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            if (s.requestId < 0 || s.requestId >= instance.nbRequests()) {
                log.add("stop with an invalid request id", "route " + std::to_string(r) + " pos " + std::to_string(i));
                continue;
            }
            const Request& request = instance.request(s.requestId);
            int t = s.ltwDarp;
            stopsByRequest[s.requestId].push_back({s.role, r, i, s.pointId, t});

            if (s.serviceTime != request.loadingTime()) {
                log.add("service time differs from the request's loading time",
                        req(s.requestId) + " at " + at({s.role, r, i, s.pointId, t}) + ": " +
                            std::to_string(s.serviceTime) + " instead of " + std::to_string(request.loadingTime()));
            }
            int earliest = prevTime + prevService + tt.driveTime(prevPoint, s.pointId);
            if (t < earliest) {
                log.add("vehicle cannot reach a stop that early",
                        "route " + std::to_string(r) + " pos " + std::to_string(i) + ": t=" + std::to_string(t) +
                            " < " + std::to_string(earliest));
            }
            if (t < 0 || t > horizon) {
                log.add("service time outside [0;horizon]",
                        "route " + std::to_string(r) + " pos " + std::to_string(i) + ": t=" + std::to_string(t));
            }

            bool boarding = s.role == StopRole::Pickup || s.role == StopRole::StopAfter;
            load += boarding ? request.quantity() : -request.quantity();
            if (load > route.capacity() || load < 0) {
                log.add("capacity violated", "route " + std::to_string(r) + " after pos " + std::to_string(i) +
                                                 ": load " + std::to_string(load));
            }

            prevPoint = s.pointId;
            prevTime = t;
            prevService = request.loadingTime();  // the model's service time, not the stored one
        }

        if (!route.empty()) {
            int back = prevTime + prevService + tt.driveTime(prevPoint, route.depotPointId());
            if (back > route.depotReturnDeadline()) {
                log.add("vehicle back at the depot after the horizon",
                        "route " + std::to_string(r) + ": back at " + std::to_string(back) + " > " +
                            std::to_string(route.depotReturnDeadline()));
            }
            if (load != 0) log.add("vehicle not empty at the end of its route", "route " + std::to_string(r));
        }
    }

    // --- Per request: assignment, pattern, windows, ride time, synchronisation. ---
    int nbUnassigned = 0;
    int nbDirect = 0, nbWalkAtPickup = 0, nbWalkAtDelivery = 0, nbTransfer = 0;
    for (int requestId = 0; requestId < instance.nbRequests(); ++requestId) {
        const Request& request = instance.request(requestId);
        bool markedUnassigned = contains(scheduled.unassignedRequests(), requestId);
        auto found = stopsByRequest.find(requestId);

        if (found == stopsByRequest.end()) {
            if (markedUnassigned) {
                ++nbUnassigned;
            } else {
                log.add("request neither in a route nor marked unassigned", req(requestId));
            }
            continue;
        }
        if (markedUnassigned) log.add("request both in a route and marked unassigned", req(requestId));

        std::map<StopRole, std::vector<PlacedStop>> byRole;
        for (const PlacedStop& s : found->second) byRole[s.role].push_back(s);
        for (const auto& [role, list] : byRole) {
            if (list.size() > 1) log.add("request has the same role twice", req(requestId));
        }
        auto has = [&byRole](StopRole role) { return byRole.count(role) > 0; };
        auto get = [&byRole](StopRole role) { return byRole.at(role).front(); };
        bool p = has(StopRole::Pickup), d = has(StopRole::Delivery);
        bool sb = has(StopRole::StopBefore), sa = has(StopRole::StopAfter);
        const int load = request.loadingTime();

        if (p && d && !sb && !sa) {
            // Direct.
            ++nbDirect;
            PlacedStop pickup = get(StopRole::Pickup), delivery = get(StopRole::Delivery);
            checkOrder(log, requestId, pickup, delivery);
            if (pickup.pointId != request.pickupId() || delivery.pointId != request.deliveryId())
                log.add("pickup/delivery at the wrong point", req(requestId));
            checkWindow(log, "pickup outside its window", requestId, pickup.time, request.pickupWindow());
            checkWindow(log, "delivery outside its window", requestId, delivery.time, request.deliveryWindow());
            checkRideTime(log, requestId, delivery.time - pickup.time, request.maxRideTime());

        } else if (p && sb && !sa && !d) {
            // Walk at delivery: pickup -> (vehicle) -> stop -> (PT) -> nearest -> (walk) -> delivery.
            ++nbWalkAtDelivery;
            const WalkAccess& walk = request.walkAccess();
            PlacedStop pickup = get(StopRole::Pickup), stop = get(StopRole::StopBefore);
            if (walk.side() != WalkSide::AtDelivery) {
                log.add("walk-at-delivery pattern for a request that cannot walk there", req(requestId));
                continue;
            }
            checkOrder(log, requestId, pickup, stop);
            if (pickup.pointId != request.pickupId()) log.add("pickup/delivery at the wrong point", req(requestId));
            if (!contains(walk.complementaryStops(), stop.pointId))
                log.add("PT stop not among the request's candidates", req(requestId) + ": stop " + std::to_string(stop.pointId));
            int arrival = stop.time + load + tt.publicTransportTime(stop.pointId, walk.nearestStop()) + walk.walkTime();
            if (tt.hasTimetable()) {
                // The first tram from the stop after the service there that
                // does not arrive before the delivery window.
                int tram = tt.earliestArrival(stop.pointId, walk.nearestStop(), stop.time + load,
                                              TravelTimeProvider::kNoTram,
                                              request.deliveryWindow().lowerBound - walk.walkTime());
                if (tram == TravelTimeProvider::kNoTram) {
                    log.add("walk: no tram from the vehicle's stop", req(requestId));
                    continue;
                }
                arrival = tram + walk.walkTime();
            }
            checkWindow(log, "pickup outside its window", requestId, pickup.time, request.pickupWindow());
            checkWindow(log, "walk: arrival at delivery outside its window", requestId, arrival,
                        request.deliveryWindow());
            checkRideTime(log, requestId, arrival - pickup.time, request.maxRideTime());

        } else if (sa && d && !p && !sb) {
            // Walk at pickup: pickup -> (walk) -> nearest -> (PT) -> stop -> (vehicle) -> delivery.
            ++nbWalkAtPickup;
            const WalkAccess& walk = request.walkAccess();
            PlacedStop stop = get(StopRole::StopAfter), delivery = get(StopRole::Delivery);
            if (walk.side() != WalkSide::AtPickup) {
                log.add("walk-at-pickup pattern for a request that cannot walk there", req(requestId));
                continue;
            }
            checkOrder(log, requestId, stop, delivery);
            if (delivery.pointId != request.deliveryId()) log.add("pickup/delivery at the wrong point", req(requestId));
            if (!contains(walk.complementaryStops(), stop.pointId))
                log.add("PT stop not among the request's candidates", req(requestId) + ": stop " + std::to_string(stop.pointId));
            int departure = stop.time - walk.walkTime() - tt.publicTransportTime(walk.nearestStop(), stop.pointId);
            if (tt.hasTimetable()) {
                // The last tram reaching the vehicle's stop by its service
                // there that can be caught leaving within the pickup window.
                int tram = tt.latestDeparture(walk.nearestStop(), stop.pointId, stop.time,
                                              request.pickupWindow().upperBound + walk.walkTime());
                if (tram == -TravelTimeProvider::kNoTram) {
                    log.add("walk: no tram to the vehicle's stop", req(requestId));
                    continue;
                }
                departure = tram - walk.walkTime();
            }
            checkWindow(log, "walk: departure from pickup outside its window", requestId, departure,
                        request.pickupWindow());
            checkWindow(log, "delivery outside its window", requestId, delivery.time, request.deliveryWindow());
            checkRideTime(log, requestId, delivery.time - departure, request.maxRideTime());

        } else if (p && sb && sa && d) {
            // Two vehicles: pickup -> (A) -> stopBefore -> (PT) -> stopAfter -> (B) -> delivery.
            ++nbTransfer;
            const PtLegAccess& pt = request.ptAccess();
            PlacedStop pickup = get(StopRole::Pickup), stopBefore = get(StopRole::StopBefore);
            PlacedStop stopAfter = get(StopRole::StopAfter), delivery = get(StopRole::Delivery);
            checkOrder(log, requestId, pickup, stopBefore);
            checkOrder(log, requestId, stopAfter, delivery);
            if (pickup.route == stopAfter.route) log.add("both legs of a transfer on the same vehicle", req(requestId));
            if (pickup.pointId != request.pickupId() || delivery.pointId != request.deliveryId())
                log.add("pickup/delivery at the wrong point", req(requestId));
            bool candidatePair = std::any_of(pt.pairs().begin(), pt.pairs().end(), [&](const StopPair& p) {
                return p.stopBefore == stopBefore.pointId && p.stopAfter == stopAfter.pointId;
            });
            if (requireCandidatePairs && !candidatePair)
                log.add("PT stop pair not among the request's candidates",
                        req(requestId) + ": stops " + std::to_string(stopBefore.pointId) + " -> " +
                            std::to_string(stopAfter.pointId));

            int ptTime = tt.publicTransportTime(stopBefore.pointId, stopAfter.pointId);
            if (ptTime < tt.minimumTimeInTransit())
                log.add("transfer: PT ride shorter than the minimum time in transit",
                        req(requestId) + ": stops " + std::to_string(stopBefore.pointId) + " -> " +
                            std::to_string(stopAfter.pointId) + ", pt " + std::to_string(ptTime) + " < " +
                            std::to_string(tt.minimumTimeInTransit()));
            auto transfer = std::find_if(scheduled.transfers().begin(), scheduled.transfers().end(),
                                         [requestId](const Transfer& t) { return t.requestId == requestId; });
            if (transfer == scheduled.transfers().end()) {
                log.add("transfer pattern without a Transfer record", req(requestId));
            } else {
                if (transfer->ptTravelTime != ptTime)
                    log.add("Transfer record has a wrong PT time",
                            req(requestId) + ": " + std::to_string(transfer->ptTravelTime) + " instead of " +
                                std::to_string(ptTime));
                if (transfer->routeBefore != pickup.route || transfer->posPickup != pickup.position ||
                    transfer->posStopBefore != stopBefore.position || transfer->routeAfter != stopAfter.route ||
                    transfer->posStopAfter != stopAfter.position || transfer->posDelivery != delivery.position)
                    log.add("Transfer record points at the wrong stops", req(requestId));
            }
            int earliestStopAfter = tt.hasTimetable()
                                        ? tt.earliestArrival(stopBefore.pointId, stopAfter.pointId, stopBefore.time + load)
                                        : stopBefore.time + load + ptTime;
            if (stopAfter.time < earliestStopAfter)
                log.add("transfer: second vehicle leaves before the request arrives",
                        req(requestId) + ": t(stopAfter)=" + std::to_string(stopAfter.time) + " < " +
                            std::to_string(earliestStopAfter));
            checkWindow(log, "pickup outside its window", requestId, pickup.time, request.pickupWindow());
            checkWindow(log, "delivery outside its window", requestId, delivery.time, request.deliveryWindow());
            checkRideTime(log, requestId, delivery.time - pickup.time, request.maxRideTime());

        } else {
            log.add("request has an incomplete or unknown set of stops", req(requestId));
        }
    }

    Solution withObjective = scheduled;
    withObjective.computeObjectiveFunction();
    out << "[checkSolution, " << label << " schedule] " << log.total() << " violation(s), " << nbUnassigned
        << " unassigned request(s), objective " << withObjective.objectiveFunction() << " -- " << nbDirect
        << " direct, " << nbWalkAtPickup << " walk at pickup, " << nbWalkAtDelivery << " walk at delivery, "
        << nbTransfer << " 2-vehicle\n";
    log.print(out);
    return log.total();
}

int checkSolution(const Solution& solution, std::ostream& out, bool requireCandidatePairs) {
    Solution scheduled = solution;
    if (!scheduled.propagateForwardDarp()) {
        out << "[checkSolution] propagateForwardDarp reports the solution infeasible -- no schedule to check\n";
        return 1;
    }
    int nbViolations = checkSchedule(scheduled, "earliest", out, requireCandidatePairs);

    if (!scheduled.propagateBackwardDarp()) {
        out << "[checkSolution] propagateBackwardDarp reports the solution infeasible although the forward pass "
               "found a schedule\n";
        return nbViolations + 1;
    }
    // The latest schedule must be feasible too, and never earlier than the
    // earliest one at any stop.
    Solution latest = scheduled;
    int nbInverted = 0;
    for (int r = 0; r < latest.nbRoutes(); ++r) {
        for (int i = 0; i < latest.route(r).size(); ++i) {
            RouteStop& s = latest.route(r).stopAt(i);
            if (s.utwDarp < s.ltwDarp) {
                if (nbInverted < 5)
                    out << "   route " << r << " pos " << i << ": latest " << s.utwDarp << " < earliest " << s.ltwDarp
                        << "\n";
                ++nbInverted;
            }
            s.ltwDarp = s.utwDarp;
        }
    }
    if (nbInverted > 0) out << "[checkSolution] " << nbInverted << " stop(s) with latest time < earliest time\n";
    return nbViolations + nbInverted + checkSchedule(latest, "latest", out, requireCandidatePairs);
}
