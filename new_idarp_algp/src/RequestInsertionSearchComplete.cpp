// TEMPORARY -- see RequestInsertionSearchComplete.h. Delete both files once
// the comparison against the Pareto-front search is done.

#include "RequestInsertionSearchComplete.h"

#include <algorithm>
#include <iostream>
#include <string>

#include "InsertionEvaluator.h"

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
}  // namespace

std::vector<RequestInsertionCandidate> findTransferInsertionComplete(const Solution& solution, int requestId) {

    const Instance& instance = solution.instance();
    const Request& req = instance.request(requestId);
    if (!req.ptAccess().available()) return {};

    InsertionEvaluator evaluator;
    RouteStop pickup = makePickupTemplate(req, requestId);
    RouteStop delivery = makeDeliveryTemplate(req, requestId);
    int maxRideTimeFirstLeg = instance.maxRideTimeFirstLeg(requestId);

    // Step 1: verify EVERY PDP-filtered task1 candidate -- every entry stop,
    // every route, every position -- not just a Pareto front.
    struct FeasibleTask1 {
        InsertionEvaluator::InsertionResult result;
        RouteStop stopBefore;
        int entryStop;
        int achievedDropoff;
    };
    std::vector<FeasibleTask1> feasibleTask1s;

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
            auto candidates = evaluator.generateCandidates(solution, r, pickup, stopBefore);

            for (const auto& c : candidates) {
                if (!evaluator.verify(solution, c, pickup, stopBefore, &maxRideTimeFirstLeg)) continue;

                int stopBeforeIndex = secondStopIndex(c.posPickup, c.posDelivery);
                Solution scratch = solution;
                scratch.insertStopAt(c.routeIndex, c.posPickup, pickup);
                scratch.insertStopAt(c.routeIndex, stopBeforeIndex, stopBefore);
                Route& scratchRoute = scratch.route(c.routeIndex);
                scratchRoute.recomputeLoad();
                std::vector<std::pair<int, Solution::RideTimeLink>> ov = {
                    {requestId, {Solution::NodeRef{c.routeIndex, c.posPickup},
                                 Solution::NodeRef{c.routeIndex, stopBeforeIndex}, maxRideTimeFirstLeg}}};
                if (!scratch.propagateForwardDarp(scratch.dependentRoutes(c.routeIndex), ov)) continue;

                feasibleTask1s.push_back({c, stopBefore, entryStop, scratchRoute.stopAt(stopBeforeIndex).ltwDarp});
            }
        }
    }
    //std::cout << "feasibleTask1s.size() Complete " << feasibleTask1s.size() << "\n";
    if (feasibleTask1s.empty()) return {};

    // Step 2: for EVERY feasible task1, try EVERY task2 candidate -- every
    // exit stop, every other route, every position -- and keep the true
    // global best total cost, not just the first feasible one found.
    bool foundAny = false;
    int bestTotalCost = 0;
    RequestInsertionCandidate best;

    for (const auto& t1 : feasibleTask1s) {
        // State with the first leg committed, reused for every task2 candidate.
        int stopBeforeIndex = secondStopIndex(t1.result.posPickup, t1.result.posDelivery);
        Solution base = solution;
        base.insertStopAt(t1.result.routeIndex, t1.result.posPickup, pickup);
        base.insertStopAt(t1.result.routeIndex, stopBeforeIndex, t1.stopBefore);
        base.route(t1.result.routeIndex).recomputeLoad();
 
        for (int exitStop : req.ptAccess().stopsAfter()) {
            int ptTime = instance.travelTimes().publicTransportTime(t1.entryStop, exitStop);
            RouteStop stopAfter;
            stopAfter.role = StopRole::StopAfter;
            stopAfter.requestId = requestId;
            stopAfter.pointId = exitStop;
            stopAfter.loadDelta = req.quantity();
            stopAfter.windowLtw = t1.achievedDropoff + ptTime;
            stopAfter.windowUtw = instance.horizonTime();
 
            for (int r = 0; r < solution.nbRoutes(); ++r) {
                if (r == t1.result.routeIndex) continue;  // never the same vehicle for both legs
                auto candidates2 = evaluator.generateCandidates(solution, r, stopAfter, delivery);
 
                for (const auto& c2 : candidates2) {
                    // Test the whole request: first leg (in `base`) + this
                    // second leg + the real Transfer, true maxRideTime over
                    // the whole chain.
                    Solution trial = base;
                    int posStopAfter = c2.posPickup;
                    int posDelivery = secondStopIndex(c2.posPickup, c2.posDelivery);
                    trial.insertStopAt(c2.routeIndex, posStopAfter, stopAfter);
                    trial.insertStopAt(c2.routeIndex, posDelivery, delivery);
                    Route& route2 = trial.route(c2.routeIndex);
                    route2.recomputeLoad();
                    if (!route2.hasFreeCapacity(0, route2.size() - 1, 0)) continue;
 
                    Transfer tr;
                    tr.requestId = requestId;
                    tr.routeBefore = t1.result.routeIndex;
                    tr.posPickup = t1.result.posPickup;
                    tr.posStopBefore = stopBeforeIndex;
                    tr.routeAfter = c2.routeIndex;
                    tr.posStopAfter = posStopAfter;
                    tr.posDelivery = posDelivery;
                    tr.ptTravelTime = ptTime;
                    trial.addTransfer(tr);
                    if (!trial.propagateForwardDarp(trial.dependentRoutes(t1.result.routeIndex))) continue;
 
                    int totalCost = t1.result.cost + c2.cost;
                    if (!foundAny || totalCost < bestTotalCost) {
                        foundAny = true;
                        bestTotalCost = totalCost;
                        best.requestId = requestId;
                        best.isTransfer = true;
                        best.cost = totalCost;
                        best.leg1 = {t1.result.routeIndex, pickup, t1.stopBefore, t1.result.posPickup,
                                     t1.result.posDelivery};
                        best.leg2 = {c2.routeIndex, stopAfter, delivery, c2.posPickup, c2.posDelivery};
                        best.ptTravelTime = ptTime;
                    }
                }
            }
        }
    }
    if (!foundAny) return {};
    return {best};
}

std::vector<RequestInsertionCandidate> findTransferInsertionWrap(const Solution& solution, int requestId) {
    RequestInsertionSearch search;

    auto byShift = search.findTransferInsertion(solution, requestId, false);
    //auto byDropoff = search.findTransferInsertion(solution, requestId, true);

    //int shiftCost = byShift.empty() ? -1 : byShift[0].cost;
    if(!byShift.empty())
        return byShift;
    else {
        auto complete = findTransferInsertionComplete(solution, requestId);
        return complete;
    }
    //int dropoffCost = byDropoff.empty() ? -1 : byDropoff[0].cost;
    

    //auto show = [](int cost) { return cost == -1 ? std::string("no feasible transfer") : std::to_string(cost); };

    //std::cout << shiftCost << " " << completeCost << "\n"; // << " " << dropoffCost 
    
    /*std::cout << "=== findTransferInsertionWrap: request " << requestId << " ===\n";
    std::cout << "  front (max shift)    : " << show(shiftCost) << "\n";
    std::cout << "  front (drop-off time): " << show(dropoffCost) << "\n";
    std::cout << "  exhaustive reference : " << show(completeCost) << "\n";*/

    /*if (completeCost != -1) {
        if (shiftCost == -1) {
            std::cout << "  -> front (max shift) found NO transfer while one exists\n";
        } else if (shiftCost != completeCost) {
            std::cout << "  -> front (max shift) MISSED the optimum by " << (shiftCost - completeCost) << "\n";
        }
        if (dropoffCost == -1) {
            std::cout << "  -> front (drop-off time) found NO transfer while one exists\n";
        } else if (dropoffCost != completeCost) {
            std::cout << "  -> front (drop-off time) MISSED the optimum by " << (dropoffCost - completeCost) << "\n";
        }
    }*/
    /*std::cout << "  stats: task1CandidatesGenerated=" << stats.task1CandidatesGenerated
              << " task1FeasibleFound=" << stats.task1FeasibleFound
              << " combinationsFeasible=" << stats.combinationsFeasible << "\n";*/
    
}
