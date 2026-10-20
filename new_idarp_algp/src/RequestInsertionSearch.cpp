#include "RequestInsertionSearch.h"

#include <algorithm>

// TEMPORARY -- wires findInsertions() to the comparison wrapper instead of
// calling findTransferInsertion() directly, so every real call site prints
// the front-vs-exhaustive comparison for free. Revert this include and the
// call in findInsertions() back to findTransferInsertion(...) once
// RequestInsertionSearchComplete.* is deleted.
#include "RequestInsertionSearchComplete.h"

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

int secondStopIndex(int posFirst, int posSecond) {
    return (posSecond == posFirst) ? posFirst + 1 : posSecond + 1;
}

bool costLess(const InsertionEvaluator::InsertionResult& a, const InsertionEvaluator::InsertionResult& b) {
    return a.cost < b.cost;
}
}  // namespace

std::vector<RequestInsertionCandidate> RequestInsertionSearch::findDirectInsertions(
    const Solution& solution, int requestId, int maxResults) const {
    const Request& req = solution.instance().request(requestId);
    RouteStop pickup = makePickupTemplate(req, requestId);
    RouteStop delivery = makeDeliveryTemplate(req, requestId);

    // Merge PDP-filtered candidates from every route BEFORE running any
    // expensive exact check, so verification happens in true global cost
    // order -- never wasted on a route whose candidates are all pricier than
    // one already found feasible elsewhere.
    std::vector<InsertionEvaluator::InsertionResult> allCandidates;
    for (int r = 0; r < solution.nbRoutes(); ++r) {
        auto cands = evaluator_.generateCandidates(solution, r, pickup, delivery);
        allCandidates.insert(allCandidates.end(), cands.begin(), cands.end());
    }
    std::sort(allCandidates.begin(), allCandidates.end(), costLess);

    std::vector<RequestInsertionCandidate> pool;
    for (const auto& c : allCandidates) {
        if (!evaluator_.verify(solution, c, pickup, delivery)) continue;
        RequestInsertionCandidate rc;
        rc.requestId = requestId;
        rc.cost = c.cost;
        rc.isTransfer = false;
        rc.leg1 = {c.routeIndex, pickup, delivery, c.posPickup, c.posDelivery};
        pool.push_back(rc);
        if (static_cast<int>(pool.size()) >= maxResults) break;
    }
    return pool;
}

std::vector<RequestInsertionCandidate> RequestInsertionSearch::findWalkInsertions(
    const Solution& solution, int requestId, int maxResults) const {
    std::vector<RequestInsertionCandidate> pool;
    const Request& req = solution.instance().request(requestId);
    if (!req.walkAccess().available()) return pool;

    struct TaggedCandidate {
        InsertionEvaluator::InsertionResult result;
        RouteStop nearStop;
        RouteStop farStop;
    };
    std::vector<TaggedCandidate> allCandidates;

    for (int stopId : req.walkAccess().complementaryStops()) {
        TimeWindow w = req.walkAccess().windowForStop(stopId);
        RouteStop nearStop, farStop;

        if (req.walkAccess().side() == WalkSide::AtPickup) {
            nearStop.role = StopRole::StopAfter;
            nearStop.requestId = requestId;
            nearStop.pointId = stopId;
            nearStop.loadDelta = req.quantity();
            nearStop.windowLtw = w.lowerBound;
            nearStop.windowUtw = w.upperBound;
            farStop = makeDeliveryTemplate(req, requestId);
        } else {
            nearStop = makePickupTemplate(req, requestId);
            farStop.role = StopRole::StopBefore;
            farStop.requestId = requestId;
            farStop.pointId = stopId;
            farStop.loadDelta = -req.quantity();
            farStop.windowLtw = w.lowerBound;
            farStop.windowUtw = w.upperBound;
        }

        for (int r = 0; r < solution.nbRoutes(); ++r) {
            auto cands = evaluator_.generateCandidates(solution, r, nearStop, farStop);
            for (auto& c : cands) allCandidates.push_back({c, nearStop, farStop});
        }
    }

    std::sort(allCandidates.begin(), allCandidates.end(),
              [](const TaggedCandidate& a, const TaggedCandidate& b) { return a.result.cost < b.result.cost; });

    for (const auto& tc : allCandidates) {
        if (!evaluator_.verify(solution, tc.result, tc.nearStop, tc.farStop)) continue;
        RequestInsertionCandidate rc;
        rc.requestId = requestId;
        rc.cost = tc.result.cost;
        rc.isTransfer = false;
        rc.leg1 = {tc.result.routeIndex, tc.nearStop, tc.farStop, tc.result.posPickup, tc.result.posDelivery};
        pool.push_back(rc);
        if (static_cast<int>(pool.size()) >= maxResults) break;
    }
    return pool;
}

RequestInsertionSearch::SecondLegResult RequestInsertionSearch::findBestSecondLeg(
    const Solution& solution, int requestId, const RouteStop& pickup, const RouteStop& stopBefore,
    const RouteStop& delivery, int entryStop, const InsertionEvaluator::InsertionResult& task1,
    int achievedDropoff) const {
    const Instance& instance = solution.instance();
    const Request& req = instance.request(requestId);

    // State with the first leg already committed, reused for every
    // second-leg candidate tried below (task1's own feasibility and capacity
    // were already verified by the caller).
    int stopBeforeIndex = secondStopIndex(task1.posPickup, task1.posDelivery);
    Solution base = solution;
    base.insertStopAt(task1.routeIndex, task1.posPickup, pickup);
    base.insertStopAt(task1.routeIndex, stopBeforeIndex, stopBefore);
    base.route(task1.routeIndex).recomputeLoad();

    struct StopAfterCandidate {
        InsertionEvaluator::InsertionResult result;
        RouteStop stopAfter;
        int ptTravelTime;
    };
    std::vector<StopAfterCandidate> task2Candidates;

    for (int exitStop : req.ptAccess().stopsAfter()) {
        int ptTime = instance.travelTimes().publicTransportTime(entryStop, exitStop);
        RouteStop stopAfter;
        stopAfter.role = StopRole::StopAfter;
        stopAfter.requestId = requestId;
        stopAfter.pointId = exitStop;
        stopAfter.loadDelta = req.quantity();
        stopAfter.windowLtw = achievedDropoff + ptTime;
        stopAfter.windowUtw = instance.horizonTime();  // the real constraint is enforced by the whole-request check below

        for (int r = 0; r < solution.nbRoutes(); ++r) {
            if (r == task1.routeIndex) continue;  // never the same vehicle for both legs
            auto cands = evaluator_.generateCandidates(solution, r, stopAfter, delivery);
            for (auto& c : cands) task2Candidates.push_back({c, stopAfter, ptTime});
        }
    }
    std::sort(task2Candidates.begin(), task2Candidates.end(),
              [](const auto& a, const auto& b) { return a.result.cost < b.result.cost; });

    for (const auto& tc : task2Candidates) {
        // Test the whole request: first leg (already in `base`) + this second
        // leg + the actual Transfer edge, so the true maxRideTime(request)
        // applies across the whole pickup -> delivery chain.
        Solution trial = base;
        int posStopAfter = tc.result.posPickup;
        int posDelivery = secondStopIndex(tc.result.posPickup, tc.result.posDelivery);
        trial.insertStopAt(tc.result.routeIndex, posStopAfter, tc.stopAfter);
        trial.insertStopAt(tc.result.routeIndex, posDelivery, delivery);
        Route& route2 = trial.route(tc.result.routeIndex);
        route2.recomputeLoad();
        if (!route2.hasFreeCapacity(0, route2.size() - 1, 0)) continue;

        Transfer t;
        t.requestId = requestId;
        t.routeBefore = task1.routeIndex;
        t.posPickup = task1.posPickup;
        t.posStopBefore = stopBeforeIndex;
        t.routeAfter = tc.result.routeIndex;
        t.posStopAfter = posStopAfter;
        t.posDelivery = posDelivery;
        t.ptTravelTime = tc.ptTravelTime;
        trial.addTransfer(t);

        if (!trial.propagateForwardDarp(trial.dependentRoutes(task1.routeIndex))) continue;

        return {true, tc.result, tc.stopAfter, tc.ptTravelTime};
    }
    return {};
}

std::vector<RequestInsertionCandidate> RequestInsertionSearch::findTransferInsertion(const Solution& solution,
                                                                                       int requestId,
                                                                                       bool useDropoffMetric,
                                                                                       TransferFrontStats* outStats) const {
    if (outStats) *outStats = TransferFrontStats{};
    const Instance& instance = solution.instance();
    const Request& req = instance.request(requestId);
    if (!req.ptAccess().available()) return {};

    RouteStop pickup = makePickupTemplate(req, requestId);
    RouteStop delivery = makeDeliveryTemplate(req, requestId);
    int maxRideTimeFirstLeg = instance.maxRideTimeFirstLeg(requestId);

    // Step 1: per-route Pareto front (cost, metric) for the first leg
    // (pickup -> stopBefore), across every entry stop, merged across routes
    // -- "front per route, then merge": each route's front is already
    // Pareto-optimal on its own, so adding its points into the global front
    // and re-filtering dominance there gives exactly the true global front.
    struct Task1FrontEntry {
        InsertionEvaluator::FrontCandidate candidate;
        RouteStop stopBefore;
        int entryStop;
    };
    ParetoFront2D<Task1FrontEntry> globalFront;

    for (int entryStop : req.ptAccess().stopsBefore()) {
        TimeWindow w = req.ptAccess().windowForStopBefore(entryStop);
        RouteStop stopBefore;
        stopBefore.role = StopRole::StopBefore;
        stopBefore.requestId = requestId;
        stopBefore.pointId = entryStop;
        stopBefore.loadDelta = -req.quantity();
        stopBefore.windowLtw = w.lowerBound;
        stopBefore.windowUtw = w.upperBound;

        for (int r = 0; r < solution.nbRoutes(); ++r) {
            auto routeFront = evaluator_.generateFrontCandidates(solution, r, pickup, stopBefore, useDropoffMetric);
            for (auto& fc : routeFront) globalFront.add({fc, stopBefore, entryStop}, fc.result.cost, fc.metric);
        }
    }
    if (outStats) outStats->task1FrontSize = static_cast<int>(globalFront.points().size());
    if (globalFront.empty()) return {};

    // Verify every front member (not just the cheapest) -- that's the whole
    // point of the front: a costlier-but-earlier candidate here might enable
    // a cheaper (or the only feasible) second leg. Keep every one that turns
    // out feasible, along with its actually achieved drop-off time.
    struct FeasibleTask1 {
        InsertionEvaluator::InsertionResult result;
        RouteStop stopBefore;
        int entryStop;
        int achievedDropoff;
    };
    std::vector<FeasibleTask1> feasibleTask1s;

    for (const auto& point : globalFront.points()) {
        const auto& entry = point.payload;
        const auto& res = entry.candidate.result;
        if (!evaluator_.verify(solution, res, pickup, entry.stopBefore, &maxRideTimeFirstLeg)) continue;

        int stopBeforeIndex = secondStopIndex(res.posPickup, res.posDelivery);
        Solution scratch = solution;
        scratch.insertStopAt(res.routeIndex, res.posPickup, pickup);
        scratch.insertStopAt(res.routeIndex, stopBeforeIndex, entry.stopBefore);
        Route& scratchRoute = scratch.route(res.routeIndex);
        scratchRoute.recomputeLoad();
        std::vector<std::pair<int, Solution::RideTimeLink>> ov = {
            {requestId, {Solution::NodeRef{res.routeIndex, res.posPickup},
                         Solution::NodeRef{res.routeIndex, stopBeforeIndex}, maxRideTimeFirstLeg}}};
        if (!scratch.propagateForwardDarp(scratch.dependentRoutes(res.routeIndex), ov)) continue;

        feasibleTask1s.push_back({res, entry.stopBefore, entry.entryStop, scratchRoute.stopAt(stopBeforeIndex).ltwDarp});
    }
    if (outStats) outStats->task1FeasibleFound = static_cast<int>(feasibleTask1s.size());
    if (feasibleTask1s.empty()) return {};

    // Step 2: for each feasible first leg, find its own best second leg, and
    // keep the overall cheapest total combination.
    bool foundAny = false;
    int bestTotalCost = 0;
    RequestInsertionCandidate best;

    for (const auto& t1 : feasibleTask1s) {
        SecondLegResult t2 = findBestSecondLeg(solution, requestId, pickup, t1.stopBefore, delivery, t1.entryStop,
                                                 t1.result, t1.achievedDropoff);
        if (!t2.found) continue;

        int totalCost = t1.result.cost + t2.result.cost;
        if (!foundAny || totalCost < bestTotalCost) {
            foundAny = true;
            bestTotalCost = totalCost;
            best.requestId = requestId;
            best.isTransfer = true;
            best.cost = totalCost;
            best.leg1 = {t1.result.routeIndex, pickup, t1.stopBefore, t1.result.posPickup, t1.result.posDelivery};
            best.leg2 = {t2.result.routeIndex, t2.stopAfter, delivery, t2.result.posPickup, t2.result.posDelivery};
            best.ptTravelTime = t2.ptTravelTime;
        }
    }
    if (!foundAny) return {};
    return {best};
}

std::vector<RequestInsertionCandidate> RequestInsertionSearch::findInsertions(const Solution& solution,
                                                                                int requestId, int maxResults,
                                                                                bool useDropoffMetric) const {
    (void)useDropoffMetric;  // TEMPORARY: unused while wired to findTransferInsertionWrap(), see below
    std::vector<RequestInsertionCandidate> pool;
    for (auto& c : findDirectInsertions(solution, requestId, maxResults)) pool.push_back(c);
    for (auto& c : findWalkInsertions(solution, requestId, maxResults)) pool.push_back(c);
    // TEMPORARY: routed through findTransferInsertionWrap() for the
    // front-vs-exhaustive comparison printout -- useDropoffMetric is unused
    // while this wiring is in place (the wrapper always runs and compares
    // both). Revert to findTransferInsertion(solution, requestId,
    // useDropoffMetric) once RequestInsertionSearchComplete.* is deleted.
    for (auto& c : findTransferInsertionWrap(solution, requestId)) pool.push_back(c);

    std::sort(pool.begin(), pool.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; });
    if (static_cast<int>(pool.size()) > maxResults) pool.resize(maxResults);
    return pool;
}

void RequestInsertionSearch::apply(Solution& solution, const RequestInsertionCandidate& candidate) const {
    solution.insertStopAt(candidate.leg1.routeIndex, candidate.leg1.posFirst, candidate.leg1.firstStop);
    int secondIndex1 = secondStopIndex(candidate.leg1.posFirst, candidate.leg1.posSecond);
    solution.insertStopAt(candidate.leg1.routeIndex, secondIndex1, candidate.leg1.secondStop);
    solution.route(candidate.leg1.routeIndex).recomputeLoad();

    std::vector<int> affected = solution.dependentRoutes(candidate.leg1.routeIndex);

    if (candidate.isTransfer) {
        solution.insertStopAt(candidate.leg2.routeIndex, candidate.leg2.posFirst, candidate.leg2.firstStop);
        int secondIndex2 = secondStopIndex(candidate.leg2.posFirst, candidate.leg2.posSecond);
        solution.insertStopAt(candidate.leg2.routeIndex, secondIndex2, candidate.leg2.secondStop);
        solution.route(candidate.leg2.routeIndex).recomputeLoad();

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

        for (int r : solution.dependentRoutes(candidate.leg2.routeIndex)) affected.push_back(r);
        std::sort(affected.begin(), affected.end());
        affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
    }

    solution.markAssigned(candidate.requestId);
    solution.propagateForwardDarp(affected);
}