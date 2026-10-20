#include "RequestInsertionSearch.h"

#include <algorithm>
#include <iostream>
#include <tuple>

namespace {
RouteStop makePickupTemplate(const Request& req, int requestId) {
    RouteStop s;
    s.role = StopRole::Pickup;
    s.requestId = requestId;
    s.pointId = req.pickupId();
    s.serviceTime = req.loadingTime();
    s.loadDelta = req.quantity();
    s.windowLtw = req.pickupWindow().lowerBound;
    s.windowUtw = req.pickupWindow().upperBound;
    return s;
}

RouteStop makeDeliveryTemplate(const Request& req, int requestId) {
    RouteStop s;
    s.role = StopRole::Delivery;
    s.requestId = requestId;
    s.pointId = req.deliveryId();
    s.serviceTime = req.loadingTime();
    s.loadDelta = -req.quantity();
    s.windowLtw = req.deliveryWindow().lowerBound;
    s.windowUtw = req.deliveryWindow().upperBound;
    return s;
}

// A PT stop where the request leaves (StopBefore) or boards (StopAfter) a vehicle.
RouteStop makePtStopTemplate(const Request& req, int requestId, StopRole role, int pointId, TimeWindow window) {
    RouteStop s;
    s.role = role;
    s.requestId = requestId;
    s.pointId = pointId;
    s.serviceTime = req.loadingTime();
    s.loadDelta = (role == StopRole::StopAfter) ? req.quantity() : -req.quantity();
    s.windowLtw = window.lowerBound;
    s.windowUtw = window.upperBound;
    return s;
}

int secondStopIndex(int posFirst, int posSecond) {
    return (posSecond == posFirst) ? posFirst + 1 : posSecond + 1;
}

// 0 direct, 1 walk, 2 two vehicles: at equal cost, the simplest trip for the
// request, which also ties the fewest routes together.
int patternRank(const RequestInsertionCandidate& c) {
    if (c.isTransfer) return 2;
    return (c.leg1.firstStop.role == StopRole::Pickup && c.leg1.secondStop.role == StopRole::Delivery) ? 0 : 1;
}

// Cost first; then the pattern; then routes, positions and stops, only so that
// equal-cost candidates always come out in the same order (reproducible runs).
bool candidateLess(const RequestInsertionCandidate& a, const RequestInsertionCandidate& b) {
    auto key = [](const RequestInsertionCandidate& c) {
        return std::make_tuple(c.cost, patternRank(c), c.leg1.routeIndex, c.leg1.posFirst, c.leg1.posSecond,
                               c.leg1.firstStop.pointId, c.leg1.secondStop.pointId, c.leg2.routeIndex,
                               c.leg2.posFirst, c.leg2.posSecond, c.leg2.firstStop.pointId);
    };
    return key(a) < key(b);
}

// Inserts the stop(s) of `candidate` (and its Transfer) into `solution`,
// without refreshing anything. Returns the routes changed.
std::vector<int> insertCandidate(Solution& solution, const RequestInsertionCandidate& candidate) {
    solution.insertStopAt(candidate.leg1.routeIndex, candidate.leg1.posFirst, candidate.leg1.firstStop);
    int secondIndex1 = secondStopIndex(candidate.leg1.posFirst, candidate.leg1.posSecond);
    solution.insertStopAt(candidate.leg1.routeIndex, secondIndex1, candidate.leg1.secondStop);
    if (!candidate.isTransfer) return {candidate.leg1.routeIndex};

    solution.insertStopAt(candidate.leg2.routeIndex, candidate.leg2.posFirst, candidate.leg2.firstStop);
    int secondIndex2 = secondStopIndex(candidate.leg2.posFirst, candidate.leg2.posSecond);
    solution.insertStopAt(candidate.leg2.routeIndex, secondIndex2, candidate.leg2.secondStop);

    Transfer t;
    t.requestId = candidate.requestId;
    t.routeBefore = candidate.leg1.routeIndex;
    t.posPickup = candidate.leg1.posFirst;
    t.posStopBefore = secondIndex1;
    t.routeAfter = candidate.leg2.routeIndex;
    t.posStopAfter = candidate.leg2.posFirst;
    t.posDelivery = secondIndex2;
    t.ptTravelTime = candidate.ptTravelTime;
    solution.addTransfer(t);
    return {candidate.leg1.routeIndex, candidate.leg2.routeIndex};
}
}  // namespace

void RequestInsertionSearch::collectDirect(const Solution& solution, int requestId,
                                           std::vector<RequestInsertionCandidate>& out) const {
    const Request& req = solution.instance().request(requestId);
    RouteStop pickup = makePickupTemplate(req, requestId);
    RouteStop delivery = makeDeliveryTemplate(req, requestId);

    for (int r = 0; r < solution.nbRoutes(); ++r) {
        for (const auto& c : evaluator_.generateCandidates(solution, r, pickup, delivery, req.maxRideTime())) {
            RequestInsertionCandidate rc;
            rc.requestId = requestId;
            rc.cost = c.cost;
            rc.leg1 = {c.routeIndex, pickup, delivery, c.posPickup, c.posDelivery};
            out.push_back(rc);
        }
    }
}

void RequestInsertionSearch::collectWalk(const Solution& solution, int requestId,
                                         std::vector<RequestInsertionCandidate>& out) const {
    const Request& req = solution.instance().request(requestId);
    const WalkAccess& walk = req.walkAccess();
    if (!walk.available()) return;

    for (int stopId : walk.complementaryStops()) {
        RouteStop nearStop, farStop;
        if (walk.side() == WalkSide::AtPickup) {
            nearStop = makePtStopTemplate(req, requestId, StopRole::StopAfter, stopId, walk.windowForStop(stopId));
            farStop = makeDeliveryTemplate(req, requestId);
        } else {
            nearStop = makePickupTemplate(req, requestId);
            farStop = makePtStopTemplate(req, requestId, StopRole::StopBefore, stopId, walk.windowForStop(stopId));
        }

        for (int r = 0; r < solution.nbRoutes(); ++r) {
            for (const auto& c :
                 evaluator_.generateCandidates(solution, r, nearStop, farStop, walk.maxRideTimeForStop(stopId))) {
                RequestInsertionCandidate rc;
                rc.requestId = requestId;
                rc.cost = c.cost;
                rc.leg1 = {c.routeIndex, nearStop, farStop, c.posPickup, c.posDelivery};
                out.push_back(rc);
            }
        }
    }
}

void RequestInsertionSearch::collectTransfer(const Solution& solution, int requestId,
                                             std::vector<RequestInsertionCandidate>& out) const {
    const Instance& instance = solution.instance();
    const Request& req = instance.request(requestId);
    if (!req.ptAccess().available()) return;

    RouteStop pickup = makePickupTemplate(req, requestId);
    RouteStop delivery = makeDeliveryTemplate(req, requestId);
    int maxRideTimeFirstLeg = instance.maxRideTimeFirstLeg(requestId);
    int maxRideTimeLastLeg = instance.maxRideTimeLastLeg(requestId);

    for (int entryStop : req.ptAccess().stopsBefore()) {
        RouteStop stopBefore = makePtStopTemplate(req, requestId, StopRole::StopBefore, entryStop,
                                                  req.ptAccess().windowForStopBefore(entryStop));
        std::vector<InsertionEvaluator::InsertionResult> firstLegs;
        for (int r = 0; r < solution.nbRoutes(); ++r) {
            auto cands = evaluator_.generateCandidates(solution, r, pickup, stopBefore, maxRideTimeFirstLeg);
            firstLegs.insert(firstLegs.end(), cands.begin(), cands.end());
        }
        if (firstLegs.empty()) continue;

        for (int exitStop : req.ptAccess().stopsAfter()) {
            // stopsBefore/stopsAfter are separate lists: not every combination is a valid pair.
            if (!instance.isValidStopPair(requestId, entryStop, exitStop)) continue;
            int ptTime = instance.travelTimes().publicTransportTime(entryStop, exitStop);
            RouteStop stopAfter = makePtStopTemplate(req, requestId, StopRole::StopAfter, exitStop,
                                                     instance.stopAfterWindow(requestId, entryStop, exitStop));
            std::vector<InsertionEvaluator::InsertionResult> lastLegs;
            for (int r = 0; r < solution.nbRoutes(); ++r) {
                auto cands = evaluator_.generateCandidates(solution, r, stopAfter, delivery, maxRideTimeLastLeg);
                lastLegs.insert(lastLegs.end(), cands.begin(), cands.end());
            }

            for (const auto& first : firstLegs) {
                for (const auto& last : lastLegs) {
                    if (first.routeIndex == last.routeIndex) continue;  // never the same vehicle for both legs
                    if (!evaluator_.admitsTransfer(solution, first, pickup, stopBefore, last, stopAfter, delivery,
                                                   ptTime, req.maxRideTime()))
                        continue;
                    RequestInsertionCandidate c;
                    c.requestId = requestId;
                    c.cost = first.cost + last.cost;
                    c.isTransfer = true;
                    c.leg1 = {first.routeIndex, pickup, stopBefore, first.posPickup, first.posDelivery};
                    c.leg2 = {last.routeIndex, stopAfter, delivery, last.posPickup, last.posDelivery};
                    c.ptTravelTime = ptTime;
                    out.push_back(c);
                }
            }
        }
    }
}

bool RequestInsertionSearch::verifyCandidate(const Solution& solution,
                                             const RequestInsertionCandidate& candidate) const {
    Solution trial = solution;  // cheap: value types, no manual undo needed
    for (int r : insertCandidate(trial, candidate)) {
        Route& route = trial.route(r);
        route.recomputeLoad();
        if (!route.hasFreeCapacity(0, route.size() - 1, 0)) return false;
    }
    return trial.propagateForwardDarp(trial.dependentRoutes(candidate.leg1.routeIndex));
}

std::vector<RequestInsertionCandidate> RequestInsertionSearch::findInsertions(const Solution& solution,
                                                                                int requestId, int maxResults) const {
    std::vector<RequestInsertionCandidate> candidates;
    collectDirect(solution, requestId, candidates);
    collectWalk(solution, requestId, candidates);
    collectTransfer(solution, requestId, candidates);
    std::sort(candidates.begin(), candidates.end(), candidateLess);

    std::vector<RequestInsertionCandidate> pool;
    for (const auto& c : candidates) {
        if (!verifyCandidate(solution, c)) continue;
        pool.push_back(c);
        if (static_cast<int>(pool.size()) >= maxResults) break;
    }
    return pool;
}

void RequestInsertionSearch::apply(Solution& solution, const RequestInsertionCandidate& candidate) const {
    std::vector<int> changed = insertCandidate(solution, candidate);
    solution.markAssigned(candidate.requestId);
    // The candidate was verified on this exact state by findInsertions(), so
    // this can only fail if verification and commit disagree -- a bug.
    if (!solution.refreshTiming(changed)) {
        std::cerr << "[RequestInsertionSearch::apply] committed candidate for request " << candidate.requestId
                  << " is infeasible -- verification and commit disagree\n";
    }
}
