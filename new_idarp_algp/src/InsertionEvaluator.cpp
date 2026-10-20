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

bool InsertionEvaluator::admitsFirstAlone(const Solution& solution, int routeIndex, int posFirst,
                                          const RouteStop& first) const {
    const LinkedRoutesTiming& tm = solution.timing(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    int i = tm.node(routeIndex, posFirst - 1);
    int afterI = tm.node(routeIndex, posFirst);
    int hFirst = std::max(first.windowLtw, tm.earliest(i) + tm.serviceTime(i) + tt.driveTime(tm.pointId(i), first.pointId));
    if (hFirst > first.windowUtw) return false;
    int delta = hFirst + first.serviceTime + tt.driveTime(first.pointId, tm.pointId(afterI)) - tm.earliest(afterI);
    return delta <= tm.forwardSlack(afterI);
}

InsertionEvaluator::LegEstimate InsertionEvaluator::estimateLeg(const Solution& solution, int routeIndex, int posFirst,
                                                                int posSecond, const RouteStop& first,
                                                                const RouteStop& second, int maxRideTime,
                                                                const LegEstimate* upstream, int firstMinTime) const {
    LegEstimate est;
    const LinkedRoutesTiming& tm = solution.timing(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    est.timing = &tm;

    // First stop, after node i (Algorithm 1, lines 1-3).
    int i = tm.node(routeIndex, posFirst - 1);
    int afterI = tm.node(routeIndex, posFirst);
    int hFirst = std::max({first.windowLtw, firstMinTime,
                           shiftedEarliest(tm, i, upstream) + tm.serviceTime(i) + tt.driveTime(tm.pointId(i), first.pointId)});
    if (hFirst > first.windowUtw) return est;

    int hSecond = 0, latestFirst = 0, latestSecond = 0;
    if (posSecond == posFirst) {
        // Second stop right after the first: a single shift, at i's old successor.
        hSecond = std::max(second.windowLtw, hFirst + first.serviceTime + tt.driveTime(first.pointId, second.pointId));
        if (hSecond > second.windowUtw) return est;
        int delta = std::max(0, hSecond + second.serviceTime + tt.driveTime(second.pointId, tm.pointId(afterI)) -
                                    tm.earliest(afterI));
        if (delta > tm.forwardSlack(afterI)) return est;
        est.shifts[0] = {afterI, delta};
        est.nbShifts = 1;
        latestSecond = std::min(second.windowUtw, tm.latest(afterI) - second.serviceTime -
                                                      tt.driveTime(second.pointId, tm.pointId(afterI)));
        latestFirst = std::min(first.windowUtw,
                               latestSecond - first.serviceTime - tt.driveTime(first.pointId, second.pointId));
    } else {
        // Shift at i's old successor (Algorithm 1, lines 4-6)...
        int deltaFirst = std::max(0, hFirst + first.serviceTime + tt.driveTime(first.pointId, tm.pointId(afterI)) -
                                         tm.earliest(afterI));
        if (deltaFirst > tm.forwardSlack(afterI)) return est;
        latestFirst = std::min(first.windowUtw, tm.latest(afterI) - first.serviceTime -
                                                    tt.driveTime(first.pointId, tm.pointId(afterI)));

        // ...propagated to j (Algorithm 2, line 4), then the second stop after j.
        int j = tm.node(routeIndex, posSecond - 1);
        int afterJ = tm.node(routeIndex, posSecond);
        int hJ = std::max(shiftedEarliest(tm, j, upstream), tm.earliest(j) + deltaFirst - tm.slackTime(afterI, j));
        hSecond = std::max(second.windowLtw, hJ + tm.serviceTime(j) + tt.driveTime(tm.pointId(j), second.pointId));
        if (hSecond > second.windowUtw) return est;
        int deltaSecond = std::max(0, hSecond + second.serviceTime + tt.driveTime(second.pointId, tm.pointId(afterJ)) -
                                          tm.earliest(afterJ));
        if (deltaSecond > tm.forwardSlack(afterJ)) return est;
        latestSecond = std::min(second.windowUtw, tm.latest(afterJ) - second.serviceTime -
                                                      tt.driveTime(second.pointId, tm.pointId(afterJ)));
        est.shifts[0] = {afterI, deltaFirst};
        est.shifts[1] = {afterJ, deltaSecond};
        est.nbShifts = 2;
    }

    // Earliest above latest: no schedule. Then the ride time between the two
    // stops is at least (earliest second) - (latest first).
    if (latestFirst < hFirst || latestSecond < hSecond) return est;
    if (hSecond - latestFirst > maxRideTime) return est;

    est.admitted = true;
    est.firstTime = hFirst;
    est.secondTime = hSecond;
    est.firstLatest = latestFirst;
    return est;
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
    int maxRideTime) const {
    const Route& route = solution.route(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    int n = route.size();

    std::vector<InsertionResult> candidates;
    for (int posPickup = 0; posPickup <= n; ++posPickup) {
        int beforeP = route.pointBefore(posPickup);
        int afterP = route.pointAfter(posPickup);

        // Delivery immediately after pickup: 3-leg formula, replacing (beforeP,afterP) directly.
        if (estimateLeg(solution, routeIndex, posPickup, posPickup, pickupTemplate, deliveryTemplate, maxRideTime)
                .admitted) {
            int adjacentCost = tt.driveTime(beforeP, pickupTemplate.pointId) +
                               tt.driveTime(pickupTemplate.pointId, deliveryTemplate.pointId) +
                               tt.driveTime(deliveryTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
            candidates.push_back({routeIndex, posPickup, posPickup, adjacentCost});
        }

        // Delivery at a later gap: independent insertion costs, using the
        // original route's neighbors (unaffected by pickup's insertion).
        if (!admitsFirstAlone(solution, routeIndex, posPickup, pickupTemplate)) continue;
        int costPickupAlone = tt.driveTime(beforeP, pickupTemplate.pointId) +
                              tt.driveTime(pickupTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
        for (int posDelivery = posPickup + 1; posDelivery <= n; ++posDelivery) {
            if (!estimateLeg(solution, routeIndex, posPickup, posDelivery, pickupTemplate, deliveryTemplate,
                             maxRideTime)
                     .admitted)
                continue;
            int beforeD = route.pointBefore(posDelivery);
            int afterD = route.pointAfter(posDelivery);
            int costDeliveryAlone = tt.driveTime(beforeD, deliveryTemplate.pointId) +
                                    tt.driveTime(deliveryTemplate.pointId, afterD) - tt.driveTime(beforeD, afterD);
            candidates.push_back({routeIndex, posPickup, posDelivery, costPickupAlone + costDeliveryAlone});
        }
    }
    return candidates;
}

std::vector<InsertionEvaluator::FrontCandidate> InsertionEvaluator::generateFrontCandidates(
    const Solution& solution, int routeIndex, const RouteStop& pickupTemplate, const RouteStop& secondTemplate,
    bool useDropoffMetric, int maxRideTime) const {
    const Route& route = solution.route(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    int n = route.size();

    ParetoFront2D<InsertionResult> front;
    auto metricOf = [useDropoffMetric](const LegEstimate& est) {
        if (useDropoffMetric) return est.secondTime;
        int shift = 0;
        for (int k = 0; k < est.nbShifts; ++k) shift = std::max(shift, est.shifts[k].delta);
        return shift;
    };

    for (int posPickup = 0; posPickup <= n; ++posPickup) {
        int beforeP = route.pointBefore(posPickup);
        int afterP = route.pointAfter(posPickup);

        LegEstimate adjacent =
            estimateLeg(solution, routeIndex, posPickup, posPickup, pickupTemplate, secondTemplate, maxRideTime);
        if (adjacent.admitted) {
            int cost = tt.driveTime(beforeP, pickupTemplate.pointId) +
                       tt.driveTime(pickupTemplate.pointId, secondTemplate.pointId) +
                       tt.driveTime(secondTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
            front.add({routeIndex, posPickup, posPickup, cost}, cost, metricOf(adjacent));
        }

        if (!admitsFirstAlone(solution, routeIndex, posPickup, pickupTemplate)) continue;
        int costPickupAlone = tt.driveTime(beforeP, pickupTemplate.pointId) +
                              tt.driveTime(pickupTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
        for (int posSecond = posPickup + 1; posSecond <= n; ++posSecond) {
            LegEstimate est =
                estimateLeg(solution, routeIndex, posPickup, posSecond, pickupTemplate, secondTemplate, maxRideTime);
            if (!est.admitted) continue;
            int beforeS = route.pointBefore(posSecond);
            int afterS = route.pointAfter(posSecond);
            int costSecondAlone = tt.driveTime(beforeS, secondTemplate.pointId) +
                                  tt.driveTime(secondTemplate.pointId, afterS) - tt.driveTime(beforeS, afterS);
            int cost = costPickupAlone + costSecondAlone;
            front.add({routeIndex, posPickup, posSecond, cost}, cost, metricOf(est));
        }
    }

    std::vector<FrontCandidate> result;
    result.reserve(front.points().size());
    for (const auto& p : front.points()) result.push_back({p.payload, p.secondary});
    return result;
}

bool InsertionEvaluator::verify(const Solution& solution, const InsertionResult& candidate,
                                 const RouteStop& pickupTemplate, const RouteStop& deliveryTemplate,
                                 const int* overrideMaxRideTime) const {
    Solution trial = solution;  // cheap: value types, no manual undo needed

    trial.insertStopAt(candidate.routeIndex, candidate.posPickup, pickupTemplate);
    int deliveryIndex =
        (candidate.posDelivery == candidate.posPickup) ? candidate.posPickup + 1 : candidate.posDelivery + 1;
    trial.insertStopAt(candidate.routeIndex, deliveryIndex, deliveryTemplate);

    Route& trialRoute = trial.route(candidate.routeIndex);
    trialRoute.recomputeLoad();
    if (!trialRoute.hasFreeCapacity(0, trialRoute.size() - 1, 0)) return false;

    std::vector<std::pair<int, Solution::RideTimeLink>> overrides;
    if (overrideMaxRideTime) {
        Solution::NodeRef nearNode{candidate.routeIndex, candidate.posPickup};
        Solution::NodeRef farNode{candidate.routeIndex, deliveryIndex};
        overrides.push_back({pickupTemplate.requestId, {nearNode, farNode, *overrideMaxRideTime}});
    }

    std::vector<int> affected = trial.dependentRoutes(candidate.routeIndex);
    return trial.propagateForwardDarp(affected, overrides);
}
