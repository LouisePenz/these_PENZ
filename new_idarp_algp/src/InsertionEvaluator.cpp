#include "InsertionEvaluator.h"

#include <algorithm>
#include <iostream>

std::vector<InsertionEvaluator::InsertionResult> InsertionEvaluator::generateCandidates(
    const Solution& solution, int routeIndex, const RouteStop& pickupTemplate,
    const RouteStop& deliveryTemplate) const {
    const Route& route = solution.route(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    int n = route.size();

    // PDP preprocessing (Masson, Lehuédé, Péton), local to this route only --
    // valid as a pre-filter regardless of pattern, but only exact/complete
    // when this route has no active Transfer (ignores ride-time-max and any
    // cross-route arc on purpose). A copy: recomputePdpWindows/ForwardTimeSlack
    // mutate ltwPdp/utwPdp/forwardTimeSlackPdp, and `route` here is a
    // reference into `solution`, which this function must not mutate.
    Route routeWithPdp = route;
    routeWithPdp.recomputePdpWindows(tt);
    routeWithPdp.recomputeForwardTimeSlack(tt);

    std::vector<InsertionResult> candidates;
    candidates.reserve(static_cast<size_t>(n + 1) * (n + 2) / 2);

    for (int posPickup = 0; posPickup <= n; ++posPickup) {
        int beforeP = route.pointBefore(posPickup);
        int afterP = route.pointAfter(posPickup);

        bool pickupAdmitted = routeWithPdp.pdpAdmitsInsertion(
            tt, posPickup, pickupTemplate.pointId, pickupTemplate.serviceTime, pickupTemplate.windowLtw,
            pickupTemplate.windowUtw);
        /*if (pickupAdmitted) {
            std::cout << "Pickup can be inserted after " << beforeP << "\n";  // A SUPPRIMER
        } else {
            std::cout << "Pickup cannot be inserted after " << beforeP << " (PDP filter)\n";  // A SUPPRIMER
            continue;  // rules out this posPickup entirely, adjacent case included
        }*/
        if (!pickupAdmitted) {
            continue;  // rules out this posPickup entirely, adjacent case included
        }

        int costPickupAlone = tt.driveTime(beforeP, pickupTemplate.pointId) +
                               tt.driveTime(pickupTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);

        // Delivery immediately after pickup: 3-leg formula, replacing (beforeP,afterP) directly.
        int adjacentCost = tt.driveTime(beforeP, pickupTemplate.pointId) +
                            tt.driveTime(pickupTemplate.pointId, deliveryTemplate.pointId) +
                            tt.driveTime(deliveryTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
        candidates.push_back({routeIndex, posPickup, posPickup, adjacentCost});

        // Delivery at a later gap: independent insertion cost, using the
        // original route's neighbors (unaffected by pickup's insertion). The
        // PDP filter is applied against the ORIGINAL route (pickup absent) --
        // still a valid necessary condition, since inserting pickup can only
        // ever add delay, never remove it (see the discussion on why).
        for (int posDelivery = posPickup + 1; posDelivery <= n; ++posDelivery) {
            int beforeD = route.pointBefore(posDelivery);
            int afterD = route.pointAfter(posDelivery);

            bool deliveryAdmitted = routeWithPdp.pdpAdmitsInsertion(
                tt, posDelivery, deliveryTemplate.pointId, deliveryTemplate.serviceTime, deliveryTemplate.windowLtw,
                deliveryTemplate.windowUtw);
            /*if (deliveryAdmitted) {
                std::cout << "Delivery can be inserted after " << beforeD << "\n";  // A SUPPRIMER
            } else {
                std::cout << "Delivery cannot be inserted after " << beforeD << " (PDP filter)\n";  // A SUPPRIMER
                continue;
            }*/
           if (!deliveryAdmitted) {
                continue;
            }

            int costDeliveryAlone = tt.driveTime(beforeD, deliveryTemplate.pointId) +
                                     tt.driveTime(deliveryTemplate.pointId, afterD) - tt.driveTime(beforeD, afterD);
            candidates.push_back({routeIndex, posPickup, posDelivery, costPickupAlone + costDeliveryAlone});
        }
    }

    return candidates;
}

std::vector<InsertionEvaluator::FrontCandidate> InsertionEvaluator::generateFrontCandidates(
    const Solution& solution, int routeIndex, const RouteStop& pickupTemplate, const RouteStop& secondTemplate,
    bool useDropoffMetric) const {
    const Route& route = solution.route(routeIndex);
    const TravelTimeProvider& tt = solution.instance().travelTimes();
    int n = route.size();

    Route routeWithPdp = route;
    routeWithPdp.recomputePdpWindows(tt);
    routeWithPdp.recomputeForwardTimeSlack(tt);

    ParetoFront2D<InsertionResult> front;

    for (int posPickup = 0; posPickup <= n; ++posPickup) {
        int beforeP = route.pointBefore(posPickup);
        int afterP = route.pointAfter(posPickup);

        // Adjacent case: second stop goes right after pickup, no route stop
        // in between -- needs the chained 2-stop PDP check, not two
        // independent single-stop ones (see Route::pdpAdmitsAdjacentInsertion).
        int adjacentArrival = 0, adjacentDelta = 0;
        bool adjacentAdmitted = routeWithPdp.pdpAdmitsAdjacentInsertion(
            tt, posPickup, pickupTemplate.pointId, pickupTemplate.serviceTime, pickupTemplate.windowLtw,
            pickupTemplate.windowUtw, secondTemplate.pointId, secondTemplate.serviceTime, secondTemplate.windowLtw,
            secondTemplate.windowUtw, &adjacentArrival, &adjacentDelta);
        if (adjacentAdmitted) {
            int cost = tt.driveTime(beforeP, pickupTemplate.pointId) +
                       tt.driveTime(pickupTemplate.pointId, secondTemplate.pointId) +
                       tt.driveTime(secondTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);
            int metric = useDropoffMetric ? adjacentArrival : adjacentDelta;
            front.add({routeIndex, posPickup, posPickup, cost}, cost, metric);
        }

        // Non-adjacent: pickup and the second stop at two independent gaps.
        int pickupDelta = 0;
        bool pickupAdmitted = routeWithPdp.pdpAdmitsInsertion(tt, posPickup, pickupTemplate.pointId,
                                                                pickupTemplate.serviceTime, pickupTemplate.windowLtw,
                                                                pickupTemplate.windowUtw, nullptr, &pickupDelta);
        if (!pickupAdmitted) continue;
        int costPickupAlone = tt.driveTime(beforeP, pickupTemplate.pointId) +
                               tt.driveTime(pickupTemplate.pointId, afterP) - tt.driveTime(beforeP, afterP);

        for (int posSecond = posPickup + 1; posSecond <= n; ++posSecond) {
            int beforeS = route.pointBefore(posSecond);
            int afterS = route.pointAfter(posSecond);
            int secondArrival = 0, secondDelta = 0;
            bool secondAdmitted = routeWithPdp.pdpAdmitsInsertion(
                tt, posSecond, secondTemplate.pointId, secondTemplate.serviceTime, secondTemplate.windowLtw,
                secondTemplate.windowUtw, &secondArrival, &secondDelta);
            if (!secondAdmitted) continue;

            int costSecondAlone = tt.driveTime(beforeS, secondTemplate.pointId) +
                                   tt.driveTime(secondTemplate.pointId, afterS) - tt.driveTime(beforeS, afterS);
            int cost = costPickupAlone + costSecondAlone;
            int metric = useDropoffMetric ? secondArrival : std::max(pickupDelta, secondDelta);
            front.add({routeIndex, posPickup, posSecond, cost}, cost, metric);
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

std::vector<InsertionEvaluator::InsertionResult> InsertionEvaluator::findInsertions(
    const Solution& solution, int routeIndex, const RouteStop& pickupTemplate,
    const RouteStop& deliveryTemplate, int maxResults, const int* overrideMaxRideTime) const {
    std::vector<InsertionResult> candidates =
        generateCandidates(solution, routeIndex, pickupTemplate, deliveryTemplate);
    std::sort(candidates.begin(), candidates.end(),
              [](const InsertionResult& a, const InsertionResult& b) { return a.cost < b.cost; });

    std::vector<InsertionResult> results;
    for (const InsertionResult& c : candidates) {
        if (!verify(solution, c, pickupTemplate, deliveryTemplate, overrideMaxRideTime)) continue;
        results.push_back(c);
        if (static_cast<int>(results.size()) >= maxResults) break;
    }
    return results;
}
