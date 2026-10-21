#include "InsertionEvaluator.h"

#include <algorithm>

namespace {
// Earliest time of `node` once the shifts of `upstream` (another leg of the
// same request, possibly in another set of linked routes) have propagated.
int shiftedEarliest(const LinkedRoutesTiming& timing, int node, const InsertionEvaluator::LegEstimate* upstream) {
    int h = timing.earliest(node);
    if (upstream == nullptr || upstream->timing != &timing) return h;
    for (int k = 0; k < upstream->nbShifts; ++k) {
        int st = timing.slackTime(upstream->shifts[k].node, node);
        if (st != LinkedRoutesTiming::kUnreachable) h = std::max(h, timing.earliest(node) + upstream->shifts[k].delta - st);
    }
    return h;
}
}  // namespace

InsertionEvaluator::FirstStop InsertionEvaluator::placeFirst(const Solution& solution, int routeIndex, int posFirst,
                                                            const RouteStop& first, const LegEstimate* upstream,
                                                            int firstMinTime) const {
    FirstStop f;
    const LinkedRoutesTiming& tm = solution.timing(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    f.timing = &tm;
    f.routeIndex = routeIndex;
    f.afterNode = tm.node(routeIndex, posFirst);

    // First stop, after node i (Algorithm 1, lines 1-3)...
    int i = tm.node(routeIndex, posFirst - 1);
    f.time = std::max({first.windowLtw, firstMinTime,
                       shiftedEarliest(tm, i, upstream) + tm.serviceTime(i) + tt.driveTime(tm.pointId(i), first.pointId)});
    if (f.time > first.windowUtw) return f;

    // ...and the shift at i's old successor (lines 4-6).
    f.shift = std::max(0, f.time + first.serviceTime + tt.driveTime(first.pointId, tm.pointId(f.afterNode)) -
                              tm.earliest(f.afterNode));
    if (f.shift > tm.forwardSlack(f.afterNode)) return f;
    f.latest = std::min(first.windowUtw, tm.latest(f.afterNode) - first.serviceTime -
                                             tt.driveTime(first.pointId, tm.pointId(f.afterNode)));
    f.admitted = true;
    return f;
}

InsertionEvaluator::LegEstimate InsertionEvaluator::placeSecond(const Solution& solution, const FirstStop& f,
                                                                int posSecond, const RouteStop& second,
                                                                int maxRideTime, const LegEstimate* upstream) const {
    LegEstimate est;
    const LinkedRoutesTiming& tm = *f.timing;
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    est.timing = &tm;

    // The first stop's shift propagated to j (Algorithm 2, line 4), then the second stop after j.
    int j = tm.node(f.routeIndex, posSecond - 1);
    int afterJ = tm.node(f.routeIndex, posSecond);
    int hJ = std::max(shiftedEarliest(tm, j, upstream), tm.earliest(j) + f.shift - tm.slackTime(f.afterNode, j));
    int hSecond = std::max(second.windowLtw, hJ + tm.serviceTime(j) + tt.driveTime(tm.pointId(j), second.pointId));
    if (hSecond > second.windowUtw) return est;
    int deltaSecond = std::max(0, hSecond + second.serviceTime + tt.driveTime(second.pointId, tm.pointId(afterJ)) -
                                      tm.earliest(afterJ));
    if (deltaSecond > tm.forwardSlack(afterJ)) return est;
    int latestSecond = std::min(second.windowUtw, tm.latest(afterJ) - second.serviceTime -
                                                      tt.driveTime(second.pointId, tm.pointId(afterJ)));

    // Earliest above latest: no schedule. Then the ride time between the two
    // stops is at least (earliest second) - (latest first).
    if (f.latest < f.time || latestSecond < hSecond) return est;
    if (hSecond - f.latest > maxRideTime) return est;

    est.admitted = true;
    est.firstTime = f.time;
    est.secondTime = hSecond;
    est.firstLatest = f.latest;
    est.shifts[0] = {f.afterNode, f.shift};
    est.shifts[1] = {afterJ, deltaSecond};
    est.nbShifts = 2;
    return est;
}

InsertionEvaluator::LegEstimate InsertionEvaluator::estimateLeg(const Solution& solution, int routeIndex, int posFirst,
                                                                int posSecond, const RouteStop& first,
                                                                const RouteStop& second, int maxRideTime,
                                                                const LegEstimate* upstream, int firstMinTime) const {
    if (posSecond != posFirst) {
        FirstStop f = placeFirst(solution, routeIndex, posFirst, first, upstream, firstMinTime);
        if (!f.admitted) {
            LegEstimate rejected;
            rejected.timing = f.timing;
            return rejected;
        }
        return placeSecond(solution, f, posSecond, second, maxRideTime, upstream);
    }

    // Second stop right after the first: a single shift, at i's old successor.
    LegEstimate est;
    const LinkedRoutesTiming& tm = solution.timing(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    est.timing = &tm;

    int i = tm.node(routeIndex, posFirst - 1);
    int afterI = tm.node(routeIndex, posFirst);
    int hFirst = std::max({first.windowLtw, firstMinTime,
                           shiftedEarliest(tm, i, upstream) + tm.serviceTime(i) + tt.driveTime(tm.pointId(i), first.pointId)});
    if (hFirst > first.windowUtw) return est;
    int hSecond = std::max(second.windowLtw, hFirst + first.serviceTime + tt.driveTime(first.pointId, second.pointId));
    if (hSecond > second.windowUtw) return est;
    int delta = std::max(0, hSecond + second.serviceTime + tt.driveTime(second.pointId, tm.pointId(afterI)) -
                                tm.earliest(afterI));
    if (delta > tm.forwardSlack(afterI)) return est;
    int latestSecond = std::min(second.windowUtw, tm.latest(afterI) - second.serviceTime -
                                                      tt.driveTime(second.pointId, tm.pointId(afterI)));
    int latestFirst = std::min(first.windowUtw,
                               latestSecond - first.serviceTime - tt.driveTime(first.pointId, second.pointId));

    if (latestFirst < hFirst || latestSecond < hSecond) return est;
    if (hSecond - latestFirst > maxRideTime) return est;

    est.admitted = true;
    est.firstTime = hFirst;
    est.secondTime = hSecond;
    est.firstLatest = latestFirst;
    est.shifts[0] = {afterI, delta};
    est.nbShifts = 1;
    return est;
}

bool InsertionEvaluator::admitsInRoute(const Solution& solution, int routeIndex, int posFirst, int posSecond,
                                       const RouteStop& first, const RouteStop& second, int maxRideTime) const {
    const RouteDarpTiming& rt = solution.routeTiming(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    // Nodes of the route (0 = depot departure, 1 + i = stop i): `first` goes
    // between k and k + 1, `second` between l and l + 1. Travel times include
    // the service at their origin.
    const int k = posFirst, k1 = posFirst + 1;
    const int travelKF = rt.serviceTime(k) + tt.driveTime(rt.pointId(k), first.pointId);

    // Algorithm 2, lines 1-2: first stop as early as it can be reached.
    int tF = std::max(first.windowLtw, rt.earliest(k) + travelKF);
    if (tF > first.windowUtw) return false;

    if (posSecond == posFirst) {
        // Second stop right after the first (k = l). Second as early as
        // possible; then first as late as possible given that, which also
        // gives the shortest ride between them and the latest service at k.
        const int travelFS = first.serviceTime + tt.driveTime(first.pointId, second.pointId);
        const int travelSK1 = second.serviceTime + tt.driveTime(second.pointId, rt.pointId(k1));
        int tS = std::max(second.windowLtw, tF + travelFS);
        if (tS > second.windowUtw) return false;
        int tK1 = std::max(rt.earliest(k1), tS + travelSK1);
        tF = std::min(first.windowUtw, tS - travelFS);
        if (tS - tF > maxRideTime) return false;
        int tK = std::min(rt.latest(k), tF - travelKF);
        return tK1 <= rt.latestGiven(k1, k, tK);
    }

    // Algorithm 2, lines 3-6: the shift at k + 1, with first and k as late as
    // possible given it.
    const int travelFK1 = first.serviceTime + tt.driveTime(first.pointId, rt.pointId(k1));
    int tK1 = std::max(rt.earliest(k1), tF + travelFK1);
    tF = std::min(first.windowUtw, tK1 - travelFK1);
    int tK = std::min(rt.latest(k), tF - travelKF);
    if (tK1 > rt.latestGiven(k1, k, tK)) return false;

    // Algorithm 3. When l = k + 1 (second case of Section 5.2.4), node l is
    // node k + 1 itself: tL and tK1 are the same service time.
    const int l = posSecond, l1 = posSecond + 1;
    const bool lIsK1 = (l == k1);
    const int travelLS = rt.serviceTime(l) + tt.driveTime(rt.pointId(l), second.pointId);
    const int travelSL1 = second.serviceTime + tt.driveTime(second.pointId, rt.pointId(l1));
    // Waiting between k + 1 (between `first` when l = k + 1) and `second`.
    auto waitingBeforeSecond = [&](int tS) {
        if (lIsK1) return tS - tF - travelFK1 - travelLS;
        return tS - tK1 - rt.cumulativeTravel(k1, l) - travelLS;
    };

    // Lines 1-4: earliest service times at l, second, l + 1.
    int tL = lIsK1 ? tK1 : std::max(rt.earliest(l), tK1 + rt.cumulativeTravel(k1, l));
    int tS = std::max(second.windowLtw, tL + travelLS);
    if (tS > second.windowUtw) return false;
    int tL1 = std::max(rt.earliest(l1), tS + travelSL1);
    // Line 5: l as late as possible given second and k.
    tL = std::min(rt.latestGiven(l, k, tK), tS - travelLS);
    if (lIsK1) tK1 = tL;

    // Lines 6-14: shift at l + 1, repaired if possible by serving k (and
    // first) later, which loosens the ride times of requests open at l + 1.
    int shift = tL1 - rt.latestGiven(l1, k, tK, l, tL);
    if (shift > 0) {
        if (tK + shift > rt.latest(k) || tF + shift > first.windowUtw || shift > waitingBeforeSecond(tS)) return false;
        tK += shift;
        tF += shift;
        tK1 = lIsK1 ? std::max(tK1, tF + travelFK1) : tK1 + shift;
        if (tK1 > rt.latestGiven(k1, k, tK)) return false;
        tL = std::min(rt.latestGiven(l, k, tK), tS - travelLS);
        if (lIsK1) tK1 = tL;
        if (tL1 > rt.latestGiven(l1, k, tK, l, tL)) return false;
    }

    // Lines 15-21: ride time of the inserted request, repaired if possible by
    // serving the first stop later (the second one is already as early as possible).
    int excess = tS - (tF + maxRideTime);
    if (excess > 0) {
        if (tF + excess > first.windowUtw || excess > waitingBeforeSecond(tS)) return false;
        tF += excess;
        tK1 = lIsK1 ? std::max(tK1, tF + travelFK1) : tK1 + excess;
        tK = std::min(rt.latest(k), tF - travelKF);
        if (tK1 > rt.latestGiven(k1, k, tK)) return false;
    }
    return true;
}

bool InsertionEvaluator::admitsTransfer(const Solution& solution, const InsertionResult& leg1, const RouteStop& pickup,
                                        const RouteStop& stopBefore, const InsertionResult& leg2,
                                        const RouteStop& stopAfter, const RouteStop& delivery, int ptTravelTime,
                                        int maxRideTime) const {
    // Pickup, then stopBefore (Algorithm 3, lines 1-7).
    LegEstimate legBefore = estimateLeg(solution, leg1.routeIndex, leg1.posPickup, leg1.posDelivery, pickup,
                                        stopBefore, kNoRideTimeLimit);
    if (!legBefore.admitted) return false;

    // Proposition 4: a cycle appears iff the node stopBefore follows (j) is a
    // successor of the node stopAfter precedes (sigma(i')). Only possible
    // within one set of linked routes.
    const LinkedRoutesTiming& timingAfter = solution.timing(leg2.routeIndex);
    if (legBefore.timing == &timingAfter) {
        int j = timingAfter.node(leg1.routeIndex, leg1.posDelivery - 1);
        int afterIPrime = timingAfter.node(leg2.routeIndex, leg2.posPickup);
        if (timingAfter.reaches(afterIPrime, j)) return false;
    }

    // stopAfter, no earlier than the request coming out of PT, then delivery
    // (lines 8-20), with the shifts of the first leg propagated to this route.
    LegEstimate legAfter = estimateLeg(solution, leg2.routeIndex, leg2.posPickup, leg2.posDelivery, stopAfter,
                                       delivery, kNoRideTimeLimit, &legBefore,
                                       legBefore.secondTime + stopBefore.serviceTime + ptTravelTime);
    if (!legAfter.admitted) return false;
    return legAfter.secondTime - legBefore.firstLatest <= maxRideTime;
}

std::vector<InsertionEvaluator::InsertionResult> InsertionEvaluator::generateCandidates(
    const Solution& solution, int routeIndex, const RouteStop& pickupTemplate, const RouteStop& deliveryTemplate,
    int maxRideTime, int costBound) const {
    const Route& route = solution.route(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    int n = route.size();

    // Cost of inserting the delivery alone at each gap of the original route:
    // computed once here, instead of once per pickup position in the loop below.
    thread_local std::vector<int> costDeliveryAlone;
    costDeliveryAlone.resize(n + 1);
    for (int posDelivery = 0; posDelivery <= n; ++posDelivery) {
        int beforeD = route.pointBefore(posDelivery);
        int afterD = route.pointAfter(posDelivery);
        costDeliveryAlone[posDelivery] = tt.driveTime(beforeD, deliveryTemplate.pointId) +
                                         tt.driveTime(deliveryTemplate.pointId, afterD) - tt.driveTime(beforeD, afterD);
    }

    std::vector<InsertionResult> candidates;
    for (int posPickup = 0; posPickup <= n; ++posPickup) {
        int beforeP = route.pointBefore(posPickup);
        int afterP = route.pointAfter(posPickup);

        // Delivery immediately after pickup: 3-leg formula, replacing (beforeP,afterP) directly.
        int adjacentCost = tt.driveTime(beforeP, pickupTemplate.pointId) +
                           tt.driveTime(pickupTemplate.pointId, deliveryTemplate.pointId) +
                           tt.driveTime(deliveryTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
        // Two necessary conditions: the delays propagated to the linked
        // routes (Masson et al., estimateLeg), and the route seen as a DARP
        // route with every ride time (Gschwind & Drexl, admitsInRoute).
        if (adjacentCost < costBound &&
            estimateLeg(solution, routeIndex, posPickup, posPickup, pickupTemplate, deliveryTemplate, maxRideTime)
                .admitted &&
            admitsInRoute(solution, routeIndex, posPickup, posPickup, pickupTemplate, deliveryTemplate, maxRideTime)) {
            candidates.push_back({routeIndex, posPickup, posPickup, adjacentCost});
        }

        // Delivery at a later gap: independent insertion costs, using the
        // original route's neighbors (unaffected by pickup's insertion). The
        // pickup's part of the test only depends on posPickup: done once.
        int costPickupAlone = tt.driveTime(beforeP, pickupTemplate.pointId) +
                              tt.driveTime(pickupTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
        if (costPickupAlone >= costBound) continue;
        FirstStop pickupPlaced = placeFirst(solution, routeIndex, posPickup, pickupTemplate);
        if (!pickupPlaced.admitted) continue;
        for (int posDelivery = posPickup + 1; posDelivery <= n; ++posDelivery) {
            int cost = costPickupAlone + costDeliveryAlone[posDelivery];
            if (cost >= costBound) continue;
            if (!placeSecond(solution, pickupPlaced, posDelivery, deliveryTemplate, maxRideTime).admitted) continue;
            if (!admitsInRoute(solution, routeIndex, posPickup, posDelivery, pickupTemplate, deliveryTemplate,
                               maxRideTime))
                continue;
            candidates.push_back({routeIndex, posPickup, posDelivery, cost});
        }
    }
    return candidates;
}
