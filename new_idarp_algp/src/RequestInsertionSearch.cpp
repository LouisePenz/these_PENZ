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

// The routes a candidate uses: (first leg's route, second leg's route or -1).
std::pair<int, int> routesOf(const RequestInsertionCandidate& c) {
    return {c.leg1.routeIndex, c.isTransfer ? c.leg2.routeIndex : -1};
}

// Appends `c` to `kept` unless kept is full or, with distinctRoutes, already
// holds a candidate on the same routes.
bool keepIfNew(std::vector<RequestInsertionCandidate>& kept, const RequestInsertionCandidate& c, int maxResults,
               bool distinctRoutes) {
    if (static_cast<int>(kept.size()) >= maxResults) return false;
    if (distinctRoutes && std::any_of(kept.begin(), kept.end(), [&c](const RequestInsertionCandidate& k) {
            return routesOf(k) == routesOf(c);
        }))
        return false;
    kept.push_back(c);
    return true;
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

void RequestInsertionSearch::collectTransfer(const Solution& solution, int requestId, int costBound,
                                             std::vector<RequestInsertionCandidate>& out) const {
    const Instance& instance = solution.instance();
    const Request& req = instance.request(requestId);
    if (!req.ptAccess().available()) return;

    RouteStop pickup = makePickupTemplate(req, requestId);
    RouteStop delivery = makeDeliveryTemplate(req, requestId);
    // Each pair has its own windows and leg maximum ride times, so both leg
    // lists are generated per pair. A leg costing costBound or more on its own
    // can't be part of a combination below it (the other leg costs >= 0).
    for (const StopPair& pair : req.ptAccess().pairs()) {
        RouteStop stopBefore =
            makePtStopTemplate(req, requestId, StopRole::StopBefore, pair.stopBefore, pair.windowBefore);
        std::vector<InsertionEvaluator::InsertionResult> firstLegs;
        for (int r = 0; r < solution.nbRoutes(); ++r) {
            auto cands =
                evaluator_.generateCandidates(solution, r, pickup, stopBefore, pair.maxRideTimeFirstLeg, costBound);
            firstLegs.insert(firstLegs.end(), cands.begin(), cands.end());
        }
        if (firstLegs.empty()) continue;

        RouteStop stopAfter = makePtStopTemplate(req, requestId, StopRole::StopAfter, pair.stopAfter, pair.windowAfter);
        std::vector<InsertionEvaluator::InsertionResult> lastLegs;
        for (int r = 0; r < solution.nbRoutes(); ++r) {
            auto cands =
                evaluator_.generateCandidates(solution, r, stopAfter, delivery, pair.maxRideTimeLastLeg, costBound);
            lastLegs.insert(lastLegs.end(), cands.begin(), cands.end());
        }

        for (const auto& first : firstLegs) {
            for (const auto& last : lastLegs) {
                if (first.routeIndex == last.routeIndex) continue;  // never the same vehicle for both legs
                if (first.cost + last.cost >= costBound) continue;
                if (!evaluator_.admitsTransfer(solution, first, pickup, stopBefore, last, stopAfter, delivery,
                                               pair.ptTravelTime, req.maxRideTime()))
                    continue;
                RequestInsertionCandidate c;
                c.requestId = requestId;
                c.cost = first.cost + last.cost;
                c.isTransfer = true;
                c.leg1 = {first.routeIndex, pickup, stopBefore, first.posPickup, first.posDelivery};
                c.leg2 = {last.routeIndex, stopAfter, delivery, last.posPickup, last.posDelivery};
                c.ptTravelTime = pair.ptTravelTime;
                out.push_back(c);
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
    // Warm start: `solution`'s ltwDarp are exact (refreshTiming after every
    // change), and the trial only adds this candidate's stops (ltwDarp = 0).
    return trial.propagateForwardDarp(trial.dependentRoutes(candidate.leg1.routeIndex), /*warmStart=*/true);
}

std::vector<RequestInsertionCandidate> RequestInsertionSearch::findInsertions(const Solution& solution,
                                                                                int requestId, int maxResults,
                                                                                bool distinctRoutes) const {
    // Reused from one call to the next: no reallocation once it has reached its largest size.
    thread_local std::vector<RequestInsertionCandidate> candidates;

    // 1. Direct and walk candidates (few): the maxResults best feasible ones.
    candidates.clear();
    collectDirect(solution, requestId, candidates);
    collectWalk(solution, requestId, candidates);
    std::vector<RequestInsertionCandidate> pool = verifyInOrder(solution, candidates, maxResults, distinctRoutes);

    // 2. 2-vehicle candidates (by far the most numerous): only those that can
    // still make it into the result, i.e. strictly cheaper than the worst of
    // the maxResults found in step 1 -- at equal cost a 2-vehicle candidate
    // ranks after direct and walk ones (candidateLess). With distinctRoutes
    // too: a 2-vehicle candidate never shares its routes with a direct or
    // walk one, so it can only enter the result by being cheaper than the
    // worst of step 1 as well.
    int costBound = (static_cast<int>(pool.size()) == maxResults) ? pool.back().cost
                                                                  : InsertionEvaluator::kNoCostBound;
    candidates.clear();
    collectTransfer(solution, requestId, costBound, candidates);
    for (const auto& c : verifyInOrder(solution, candidates, maxResults, distinctRoutes)) pool.push_back(c);

    // 3. The maxResults best of both (on distinct routes if asked: the best
    // ones of each step on distinct routes contain those of the union).
    std::sort(pool.begin(), pool.end(), candidateLess);
    std::vector<RequestInsertionCandidate> result;
    for (const auto& c : pool) keepIfNew(result, c, maxResults, distinctRoutes);
    return result;
}

std::vector<RequestInsertionCandidate> RequestInsertionSearch::verifyInOrder(
    const Solution& solution, std::vector<RequestInsertionCandidate>& candidates, int maxResults,
    bool distinctRoutes) const {
    // A heap rather than a full sort: only the candidates actually checked
    // are ordered, in exactly the order a sort would give (candidateLess is a
    // strict total order). std heaps put the largest on top, hence the swap.
    auto worse = [](const RequestInsertionCandidate& a, const RequestInsertionCandidate& b) {
        return candidateLess(b, a);
    };
    std::make_heap(candidates.begin(), candidates.end(), worse);

    std::vector<RequestInsertionCandidate> feasible;
    while (!candidates.empty() && static_cast<int>(feasible.size()) < maxResults) {
        std::pop_heap(candidates.begin(), candidates.end(), worse);  // cheapest moved to the back
        const RequestInsertionCandidate& c = candidates.back();
        bool sameRoutesAsFound = distinctRoutes && std::any_of(feasible.begin(), feasible.end(), [&c](const auto& f) {
                                     return routesOf(f) == routesOf(c);
                                 });
        if (!sameRoutesAsFound && verifyCandidate(solution, c)) feasible.push_back(c);
        candidates.pop_back();
    }
    return feasible;
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
