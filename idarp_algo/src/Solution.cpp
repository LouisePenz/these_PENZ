#include "Solution.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <limits>
#include <numeric>

#include "LinkedRoutesTiming.h"
#include "RouteDarpTiming.h"

namespace {
const char* roleName(StopRole role) {
    switch (role) {
        case StopRole::Pickup: return "Pickup";
        case StopRole::Delivery: return "Delivery";
        case StopRole::StopBefore: return "StopBefore";
        case StopRole::StopAfter: return "StopAfter";
    }
    return "?";
}

// Where one request's stops are in the STP graph being built (see
// Solution::buildDarpGraph): node and stop for each role, absent if null.
struct Slot {
    int node = -1;
    const RouteStop* stop = nullptr;
};
struct Placement {
    Slot pickup, delivery, stopBefore, stopAfter;
    bool transfer = false;
};
}  // namespace


Solution::Solution(const Instance& instance) : instance_(&instance) {
    routes_.reserve(instance.nbVehicles());
    for (int v = 0; v < instance.nbVehicles(); ++v) {
        // Every vehicle departs the depot at time 0 and must be back by the horizon.
        routes_.emplace_back(v, instance.capacity(), PointRegistry::depot, 0, instance.horizonTime());
    }
    unassignedRequests_.reserve(instance.nbRequests());
    for (int r = 0; r < instance.nbRequests(); ++r) unassignedRequests_.push_back(r);

    timing_.resize(routes_.size());
    routeTiming_.resize(routes_.size());
    std::vector<int> allRoutes(nbRoutes());
    std::iota(allRoutes.begin(), allRoutes.end(), 0);
    refreshTiming(allRoutes);
}

const LinkedRoutesTiming& Solution::timing(int route) const { return *timing_[route]; }

const RouteDarpTiming& Solution::routeTiming(int route) const { return *routeTiming_[route]; }

bool Solution::refreshTiming(const std::vector<int>& changedRoutes) {
    bool feasible = true;
    std::vector<bool> done(routes_.size(), false);
    for (int changed : changedRoutes) {
        if (done[changed]) continue;
        std::vector<int> linked = dependentRoutes(changed);
        std::sort(linked.begin(), linked.end());
        for (int r : linked) {
            done[r] = true;
            routes_[r].recomputeLoad();
        }
        if (!propagateForwardDarp(linked) || !propagateBackwardDarp(linked)) {
            std::cerr << "[refreshTiming] a set of linked routes has no feasible schedule\n";
            feasible = false;
        }
        auto timing = std::make_shared<const LinkedRoutesTiming>(*this, linked);
        for (int r : linked) {
            timing_[r] = timing;
            // The cubic preprocessing only when its inputs changed: a change
            // elsewhere in the set often leaves this route's windows and ride
            // time pairs exactly as they were.
            RouteDarpTiming::Inputs inputs = RouteDarpTiming::inputsOf(*this, r);
            if (!routeTiming_[r] || !(routeTiming_[r]->inputs() == inputs))
                routeTiming_[r] = std::make_shared<const RouteDarpTiming>(instance_->travelTimes(), std::move(inputs));
        }
    }
    return feasible;
}

void Solution::removeTransfer(int requestId) {
    transfers_.erase(std::remove_if(transfers_.begin(), transfers_.end(),
                                     [requestId](const Transfer& t) { return t.requestId == requestId; }),
                      transfers_.end());
}

void Solution::markAssigned(int requestId) {
    unassignedRequests_.erase(std::remove(unassignedRequests_.begin(), unassignedRequests_.end(), requestId),
                               unassignedRequests_.end());
}

std::vector<int> Solution::dependentRoutes(int start) const {
    std::unordered_set<int> visited;
    std::vector<int> frontier{start};
    visited.insert(start);
    while (!frontier.empty()) {
        int r = frontier.back();
        frontier.pop_back();
        for (const Transfer& t : transfers_) {
            int other = -1;
            if (t.routeBefore == r) other = t.routeAfter;
            else if (t.routeAfter == r) other = t.routeBefore;
            if (other != -1 && visited.insert(other).second) frontier.push_back(other);
        }
    }
    return std::vector<int>(visited.begin(), visited.end());
}

bool Solution::buildDarpGraph(const std::vector<int>& affectedRoutes, DarpGraph& graph) const {
    const int H = instance_->horizonTime();
    const TravelTimeProvider& tt = instance_->travelTimes();
    graph.routeOffset.assign(routes_.size(), -1);
    graph.nbNodes = 1;  // node 0: the artificial depot
    graph.arcs.clear();
    graph.tramArcs.clear();
    for (int r : affectedRoutes) {
        graph.routeOffset[r] = graph.nbNodes;
        graph.nbNodes += routes_[r].size() + 2;
    }
    graph.earliestStart.assign(graph.nbNodes, 0);
    graph.latestStart.assign(graph.nbNodes, H);
    for (int r : affectedRoutes) {
        graph.latestStart[graph.routeOffset[r]] = 0;  // depot departure, fixed at 0
        graph.latestStart[graph.routeOffset[r] + routes_[r].size() + 1] = routes_[r].depotReturnDeadline();
    }
    graph.earliestStart[0] = H;

    // Where each request's stops are, by role, gathered in the same single
    // pass as the route arcs -- in a workspace indexed by request, shared by
    // every call (one STP runs at a time) instead of allocated and zeroed for
    // each one; only the entries of the requests seen are reset on the way out.
    thread_local std::vector<Placement> placements;
    thread_local std::vector<int> requestsSeen;
    if (static_cast<int>(placements.size()) < instance_->nbRequests()) placements.resize(instance_->nbRequests());
    requestsSeen.clear();
    struct ResetSeen {
        ~ResetSeen() {
            for (int requestId : requestsSeen) placements[requestId] = Placement{};
        }
    } resetSeen;

    // Route chain arcs: value(succ) >= value(pred) + driveTime + serviceTime(pred).
    for (int r : affectedRoutes) {
        const Route& route = routes_[r];
        int prevNode = graph.routeOffset[r];  // depot departure
        int prevPointId = route.depotPointId();
        int prevServiceTime = 0;
        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            int node = graph.stopNode(r, i);
            graph.arcs.push_back({node, prevNode, -tt.driveTime(prevPointId, s.pointId) - prevServiceTime});
            prevNode = node;
            prevPointId = s.pointId;
            prevServiceTime = s.serviceTime;

            Placement& p = placements[s.requestId];
            bool firstSeen = !p.pickup.stop && !p.delivery.stop && !p.stopBefore.stop && !p.stopAfter.stop;
            if (firstSeen) requestsSeen.push_back(s.requestId);
            Slot& slot = (s.role == StopRole::Pickup)       ? p.pickup
                         : (s.role == StopRole::Delivery)   ? p.delivery
                         : (s.role == StopRole::StopBefore) ? p.stopBefore
                                                            : p.stopAfter;
            slot = {node, &s};
        }

        // Return to the depot: value(end) >= value(last) + serviceTime(last) +
        // driveTime(last, depot), and value(end) <= the route's return deadline.
        if (!route.empty()) {
            int endNode = graph.routeOffset[r] + route.size() + 1;
            graph.arcs.push_back({endNode, prevNode, -tt.driveTime(prevPointId, route.depotPointId()) - prevServiceTime});
            graph.arcs.push_back({0, endNode, route.depotReturnDeadline() - H});
        }
    }

    // Transfer arcs (2-vehicle case only): value(stopAfter) >= value(stopBefore)
    // + ptTravelTime + serviceTime(stopBefore). Only when both ends are in the
    // affected set -- otherwise this Transfer isn't part of this call.
    for (const Transfer& t : transfers_) {
        if (graph.routeOffset[t.routeBefore] < 0 || graph.routeOffset[t.routeAfter] < 0) continue;
        int stopBeforeServiceTime = routes_[t.routeBefore].stopAt(t.posStopBefore).serviceTime;
        graph.arcs.push_back({graph.stopNode(t.routeAfter, t.posStopAfter), graph.stopNode(t.routeBefore, t.posStopBefore),
                              -t.ptTravelTime - stopBeforeServiceTime});
        // With the timetable, the arc above only keeps the shortest ride (a
        // valid bound); the tram ride itself: leaving stopBefore after the
        // service there, reaching stopAfter by the second vehicle's service.
        if (tt.hasTimetable()) {
            const RouteStop& before = routes_[t.routeBefore].stopAt(t.posStopBefore);
            const RouteStop& after = routes_[t.routeAfter].stopAt(t.posStopAfter);
            graph.tramArcs.push_back({graph.stopNode(t.routeBefore, t.posStopBefore), stopBeforeServiceTime,
                                      graph.stopNode(t.routeAfter, t.posStopAfter), 0, before.pointId,
                                      after.pointId, TravelTimeProvider::kNoTram, -TravelTimeProvider::kNoTram});
        }
        placements[t.requestId].transfer = true;
    }

    // Per request: the two nodes its window and maximum ride time apply to --
    // pickup/delivery (direct, 2-vehicle), or the vehicle-side PT stop in
    // place of the walked end (walk patterns, with the stop's derived window
    // and leg maximum ride time) -- anchored to the artificial depot.
    for (int requestId : requestsSeen) {
        const Placement& p = placements[requestId];
        const Request& request = instance_->request(requestId);
        const WalkAccess& walk = request.walkAccess();
        auto isWalkStop = [&walk](const Slot& slot) {
            const std::vector<int>& stops = walk.complementaryStops();
            return std::find(stops.begin(), stops.end(), slot.stop->pointId) != stops.end();
        };
        bool pickup = p.pickup.stop, delivery = p.delivery.stop, before = p.stopBefore.stop, after = p.stopAfter.stop;

        Slot nearSlot, farSlot;
        int maxRideTime = 0;
        if (p.transfer && pickup && delivery && before && after) {
            nearSlot = p.pickup;
            farSlot = p.delivery;
            maxRideTime = request.maxRideTime();
        } else if (!p.transfer && pickup && delivery && !before && !after) {
            nearSlot = p.pickup;
            farSlot = p.delivery;
            maxRideTime = request.maxRideTime();
        } else if (!p.transfer && after && delivery && !pickup && !before && walk.side() == WalkSide::AtPickup &&
                   isWalkStop(p.stopAfter)) {
            nearSlot = p.stopAfter;
            farSlot = p.delivery;
            maxRideTime = walk.maxRideTimeForStop(p.stopAfter.stop->pointId);
        } else if (!p.transfer && pickup && before && !delivery && !after && walk.side() == WalkSide::AtDelivery &&
                   isWalkStop(p.stopBefore)) {
            nearSlot = p.pickup;
            farSlot = p.stopBefore;
            maxRideTime = walk.maxRideTimeForStop(p.stopBefore.stop->pointId);
        } else {
            // Skipping it would silently drop every time constraint of this
            // request -- always a bug upstream, never a legitimate state.
            std::cerr << "[buildDarpGraph] request " << requestId
                      << " is in a route but matches no known pattern -- treated as infeasible\n";
            return false;
        }

        graph.arcs.push_back({0, nearSlot.node, nearSlot.stop->windowUtw - H});
        graph.arcs.push_back({0, farSlot.node, farSlot.stop->windowUtw - H});
        graph.arcs.push_back({nearSlot.node, 0, H - nearSlot.stop->windowLtw});
        graph.arcs.push_back({farSlot.node, 0, H - farSlot.stop->windowLtw});
        graph.arcs.push_back({nearSlot.node, farSlot.node, maxRideTime});
        for (const Slot& slot : {nearSlot, farSlot}) {
            graph.earliestStart[slot.node] = std::max(0, slot.stop->windowLtw);
            graph.latestStart[slot.node] = std::min(H, slot.stop->windowUtw);
        }

        // Walk patterns with the timetable: the stop's window and leg maximum
        // ride time above are only bounds; the tram ride itself, on foot at
        // the other end (node 0 stands for the horizon H).
        if (tt.hasTimetable() && !p.transfer && (after || before)) {
            int walkTime = walk.walkTime();
            if (after) {  // walk at pickup
                // Leaves the pickup in its window, walks to the nearest stop,
                // reaches the vehicle's stop by its service there; the
                // delivery no later than the maximum ride time after leaving.
                int stop = nearSlot.stop->pointId, depCap = request.pickupWindow().upperBound + walkTime;
                graph.tramArcs.push_back({farSlot.node, walkTime - request.maxRideTime(), nearSlot.node, 0,
                                          walk.nearestStop(), stop, depCap, -TravelTimeProvider::kNoTram});
                graph.tramArcs.push_back({0, request.pickupWindow().lowerBound + walkTime - H, nearSlot.node, 0,
                                          walk.nearestStop(), stop, depCap, -TravelTimeProvider::kNoTram});
            } else {  // walk at delivery
                // Leaves the vehicle's stop after the service there, reaches
                // the delivery on foot within its window and the maximum ride
                // time after the pickup.
                int stop = farSlot.stop->pointId, arrFloor = request.deliveryWindow().lowerBound - walkTime;
                int service = farSlot.stop->serviceTime;
                graph.tramArcs.push_back({farSlot.node, service, nearSlot.node, request.maxRideTime() - walkTime,
                                          stop, walk.nearestStop(), TravelTimeProvider::kNoTram, arrFloor});
                graph.tramArcs.push_back({farSlot.node, service, 0, request.deliveryWindow().upperBound - walkTime - H,
                                          stop, walk.nearestStop(), TravelTimeProvider::kNoTram, arrFloor});
            }
        }
    }
    return true;
}

bool Solution::propagateForwardDarp(const std::vector<int>& affectedRoutes, bool warmStart) {
    const int H = instance_->horizonTime();
    // Workspaces reused from one call to the next (one STP runs at a time):
    // no allocation once they have reached their largest size.
    thread_local DarpGraph graph;
    thread_local std::vector<int> value;
    if (!buildDarpGraph(affectedRoutes, graph)) return false;

    // Earliest schedule: every node starts low and is pushed up. Start values:
    // the instance's lower bounds (graph.earliestStart) -- with warmStart,
    // raised to the stops' current ltwDarp (see the header).
    value.assign(graph.earliestStart.begin(), graph.earliestStart.end());
    if (warmStart) {
        for (int r : affectedRoutes)
            for (int i = 0; i < routes_[r].size(); ++i) {
                int node = graph.stopNode(r, i);
                value[node] = std::max(value[node], routes_[r].stopAt(i).ltwDarp);
            }
    }

    // Bellman-Ford relaxation. Values are final after |V| - 1 passes if there
    // is no positive cycle; one more pass is needed to see that nothing
    // changes anymore, hence |V| passes before declaring a cycle.
    //
    // Tram arcs (variable weights) push their late node up to the earliest
    // tram arrival. Values only increase and stay below H, so the relaxation
    // still ends -- but a chain of tram arcs may need more than |V| passes,
    // hence no pass limit with them.
    const TravelTimeProvider& tt = instance_->travelTimes();
    const int maxPasses = graph.tramArcs.empty() ? graph.nbNodes : std::numeric_limits<int>::max();
    bool converged = false;
    for (int pass = 0; pass < maxPasses && !converged; ++pass) {
        converged = true;
        for (const DarpGraph::Arc& a : graph.arcs) {
            if (value[a.to] > value[a.from] + a.weight) {
                int newValue = value[a.to] - a.weight;
                if (newValue > H) return false;  // infeasible: window exceeds the horizon
                value[a.from] = newValue;
                converged = false;
            }
        }
        for (const DarpGraph::TramArc& a : graph.tramArcs) {
            int arrival = tt.earliestArrival(a.from, a.to, value[a.early] + a.earlyOffset, a.depCap, a.arrFloor);
            if (arrival == TravelTimeProvider::kNoTram) return false;  // no tram left
            int newValue = arrival - a.lateOffset;
            if (value[a.late] < newValue) {
                if (newValue > H) return false;
                value[a.late] = newValue;
                converged = false;
            }
        }
    }
    if (!converged) return false;  // positive cycle: no assignment of times satisfies every constraint

    for (int r : affectedRoutes) {
        Route& route = routes_[r];
        for (int i = 0; i < route.size(); ++i) route.stopAt(i).ltwDarp = value[graph.stopNode(r, i)];
    }
    return true;
}

bool Solution::propagateForwardDarp(){
    std::vector<int> allRoutes(nbRoutes());
    std::iota(allRoutes.begin(), allRoutes.end(), 0);
    return this->propagateForwardDarp(allRoutes);
}

bool Solution::propagateBackwardDarp(const std::vector<int>& affectedRoutes) {
    // Workspaces reused from one call to the next, as in propagateForwardDarp.
    thread_local DarpGraph graph;
    thread_local std::vector<int> value;
    thread_local std::vector<char> fixed;
    if (!buildDarpGraph(affectedRoutes, graph)) return false;

    // Latest schedule: the mirror of propagateForwardDarp -- every node starts
    // high (the instance's upper bounds, graph.latestStart) and is pulled
    // down: value(to) <= value(from) + weight. The artificial depot (H) and
    // the route depots (departure at 0) are fixed: having to pull one of them
    // down means no schedule exists.
    value.assign(graph.latestStart.begin(), graph.latestStart.end());
    fixed.assign(graph.nbNodes, 0);
    fixed[0] = 1;
    for (int r : affectedRoutes) fixed[graph.routeOffset[r]] = 1;

    // Tram arcs pull their early node down to the latest tram departure (see
    // propagateForwardDarp for the pass limit).
    const TravelTimeProvider& tt = instance_->travelTimes();
    const int maxPasses = graph.tramArcs.empty() ? graph.nbNodes : std::numeric_limits<int>::max();
    bool converged = false;
    for (int pass = 0; pass < maxPasses && !converged; ++pass) {
        converged = true;
        for (const DarpGraph::Arc& a : graph.arcs) {
            if (value[a.to] > value[a.from] + a.weight) {
                int newValue = value[a.from] + a.weight;
                if (fixed[a.to] || newValue < 0) return false;
                value[a.to] = newValue;
                converged = false;
            }
        }
        for (const DarpGraph::TramArc& a : graph.tramArcs) {
            int departure = tt.latestDeparture(a.from, a.to, value[a.late] + a.lateOffset, a.depCap, a.arrFloor);
            if (departure == -TravelTimeProvider::kNoTram) return false;  // no tram early enough
            int newValue = departure - a.earlyOffset;
            if (value[a.early] > newValue) {
                if (fixed[a.early] || newValue < 0) return false;
                value[a.early] = newValue;
                converged = false;
            }
        }
    }
    if (!converged) return false;

    for (int r : affectedRoutes) {
        Route& route = routes_[r];
        for (int i = 0; i < route.size(); ++i) route.stopAt(i).utwDarp = value[graph.stopNode(r, i)];
    }
    return true;
}

bool Solution::propagateBackwardDarp() {
    std::vector<int> allRoutes(nbRoutes());
    std::iota(allRoutes.begin(), allRoutes.end(), 0);
    return propagateBackwardDarp(allRoutes);
}

void Solution::print() const {
    std::cout << "Solution with " << nbRoutes() << " routes, " << unassignedRequests_.size()
              << " unassigned request(s), " << transfers_.size() << " transfer(s).\n";

    for (int r = 0; r < nbRoutes(); ++r) {
        const Route& route = routes_[r];
        std::cout << "Route " << r << " (vehicle " << route.vehicleId() << ", capacity " << route.capacity()
                  << "), " << route.size() << " stop(s):\n";
        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            std::cout << "   [" << i << "] " << roleName(s.role) << " req=" << s.requestId
                      << " leg=" << s.legIndex << " point=" << s.pointId << " window=[" << s.windowLtw << ";"
                      << s.windowUtw << "] ltwDarp=" << s.ltwDarp << "\n";
        }
    }

    if (!unassignedRequests_.empty()) {
        std::cout << "Unassigned requests: ";
        for (int r : unassignedRequests_) std::cout << r << " ";
        std::cout << "\n";
    }

    if (!transfers_.empty()) {
        std::cout << "Transfers:\n";
        for (const Transfer& t : transfers_) {
            std::cout << "   req=" << t.requestId << " route " << t.routeBefore << " (pickup@"
                      << t.posPickup << ", stopBefore@" << t.posStopBefore << ") -> route " << t.routeAfter
                      << " (stopAfter@" << t.posStopAfter << ", delivery@" << t.posDelivery
                      << "), ptTravelTime=" << t.ptTravelTime << "\n";
        }
    }
}

bool Solution::checkConsistency() const {
    bool ok = true;
    auto report = [&ok](const std::string& message) {
        std::cerr << "[checkConsistency] " << message << "\n";
        ok = false;
    };

    // Every Transfer must reference two valid routes and the right role/request
    // at the right position, in the right order within each route.
    for (const Transfer& t : transfers_) {
        if (t.routeBefore < 0 || t.routeBefore >= nbRoutes() || t.routeAfter < 0 || t.routeAfter >= nbRoutes()) {
            report("Transfer for request " + std::to_string(t.requestId) + " references an out-of-range route");
            continue;
        }
        const Route& before = routes_[t.routeBefore];
        const Route& after = routes_[t.routeAfter];
        if (t.posPickup < 0 || t.posPickup >= before.size() || t.posStopBefore < 0 ||
            t.posStopBefore >= before.size() || t.posStopAfter < 0 || t.posStopAfter >= after.size() ||
            t.posDelivery < 0 || t.posDelivery >= after.size()) {
            report("Transfer for request " + std::to_string(t.requestId) + " references an out-of-range position");
            continue;
        }
        if (before.stopAt(t.posPickup).role != StopRole::Pickup || before.stopAt(t.posPickup).requestId != t.requestId)
            report("Transfer for request " + std::to_string(t.requestId) + ": posPickup is not that request's Pickup");
        if (before.stopAt(t.posStopBefore).role != StopRole::StopBefore ||
            before.stopAt(t.posStopBefore).requestId != t.requestId)
            report("Transfer for request " + std::to_string(t.requestId) + ": posStopBefore is not that request's StopBefore");
        if (after.stopAt(t.posStopAfter).role != StopRole::StopAfter ||
            after.stopAt(t.posStopAfter).requestId != t.requestId)
            report("Transfer for request " + std::to_string(t.requestId) + ": posStopAfter is not that request's StopAfter");
        if (after.stopAt(t.posDelivery).role != StopRole::Delivery || after.stopAt(t.posDelivery).requestId != t.requestId)
            report("Transfer for request " + std::to_string(t.requestId) + ": posDelivery is not that request's Delivery");
        if (t.posPickup >= t.posStopBefore)
            report("Transfer for request " + std::to_string(t.requestId) + ": pickup does not precede stopBefore in its route");
        if (t.posStopAfter >= t.posDelivery)
            report("Transfer for request " + std::to_string(t.requestId) + ": stopAfter does not precede delivery in its route");
    }

    // Every request must be either unassigned, or fully and consistently placed
    // (never both, never a partial/duplicated set of roles).
    std::unordered_map<int, std::vector<StopRole>> rolesByRequest;
    for (const Route& route : routes_) {
        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            rolesByRequest[s.requestId].push_back(s.role);
        }
    }
    std::unordered_set<int> unassignedSet(unassignedRequests_.begin(), unassignedRequests_.end());

    for (const auto& [reqId, roles] : rolesByRequest) {
        if (unassignedSet.count(reqId)) {
            report("Request " + std::to_string(reqId) + " is both placed in a route and marked unassigned");
        }
        std::vector<StopRole> sorted = roles;
        std::sort(sorted.begin(), sorted.end());
        bool isDirect = sorted == std::vector<StopRole>{StopRole::Pickup, StopRole::Delivery};
        bool isWalkAtPickup = sorted == std::vector<StopRole>{StopRole::Delivery, StopRole::StopAfter};
        bool isWalkAtDelivery = sorted == std::vector<StopRole>{StopRole::Pickup, StopRole::StopBefore};
        bool isTransfer =
            sorted == std::vector<StopRole>{StopRole::Pickup, StopRole::Delivery, StopRole::StopBefore, StopRole::StopAfter};
        if (!isDirect && !isWalkAtPickup && !isWalkAtDelivery && !isTransfer) {
            report("Request " + std::to_string(reqId) + " has an inconsistent set of stops (" +
                   std::to_string(roles.size()) + " role(s))");
        }
        if (isTransfer) {
            bool hasTransferRecord = std::any_of(transfers_.begin(), transfers_.end(),
                                                  [reqId = reqId](const Transfer& t) { return t.requestId == reqId; });
            if (!hasTransferRecord)
                report("Request " + std::to_string(reqId) + " has both vehicle legs in routes but no Transfer entry");
        }
    }
    for (int reqId : unassignedRequests_) {
        if (rolesByRequest.count(reqId))
            report("Request " + std::to_string(reqId) + " is marked unassigned but also present in a route");
    }

    // Capacity must never be exceeded anywhere in a route.
    for (int r = 0; r < nbRoutes(); ++r) {
        Route& route = const_cast<Route&>(routes_[r]);
        route.recomputeLoad();
        for (int i = 0; i < route.size(); ++i) {
            if (route.loadAfter(i) > route.capacity() || route.loadAfter(i) < 0) {
                report("Route " + std::to_string(r) + " has an invalid load (" + std::to_string(route.loadAfter(i)) +
                       ") after position " + std::to_string(i));
            }
        }
    }

    return ok;
}

void Solution::computeObjectiveFunction() {
    objectiveFunction_ = 0;
    for (const Route& route : routes_) {
        if (route.empty()) continue;
        int prevPointId = route.depotPointId();
        for (int i = 0; i < route.size(); ++i) {
            objectiveFunction_ += instance_->travelTimes().driveTime(prevPointId, route.stopAt(i).pointId);
            prevPointId = route.stopAt(i).pointId;
        }
        objectiveFunction_ += instance_->travelTimes().driveTime(prevPointId, route.depotPointId());
    }
}

void Solution::insertStopAt(int routeIndex, int position, RouteStop stop) {
    routes_[routeIndex].insertStop(position, stop);
    for (Transfer& t : transfers_) {
        if (t.routeBefore == routeIndex) {
            if (t.posPickup >= position) ++t.posPickup;
            if (t.posStopBefore >= position) ++t.posStopBefore;
        }
        if (t.routeAfter == routeIndex) {
            if (t.posStopAfter >= position) ++t.posStopAfter;
            if (t.posDelivery >= position) ++t.posDelivery;
        }
    }
}

void Solution::removeStopAt(int routeIndex, int position) {
    routes_[routeIndex].removeStop(position);
    for (Transfer& t : transfers_) {
        if (t.routeBefore == routeIndex) {
            if (t.posPickup > position) --t.posPickup;
            if (t.posStopBefore > position) --t.posStopBefore;
        }
        if (t.routeAfter == routeIndex) {
            if (t.posStopAfter > position) --t.posStopAfter;
            if (t.posDelivery > position) --t.posDelivery;
        }
    }
}

int Solution::nbUsedRoutes(){
    int nbUsedRoutes = 0;
    for (auto r: routes_) {
        if(!r.empty()){
            nbUsedRoutes++;
        }
    }
    return nbUsedRoutes;
}

void Solution::removeRequest(int requestId) {
    for (int r = 0; r < nbRoutes(); ++r) {
        Route& route = routes_[r];
        for (int i = route.size() - 1; i >= 0; --i) {
            if (route.stopAt(i).requestId == requestId) removeStopAt(r, i);
        }
    }
    removeTransfer(requestId);
    markUnassigned(requestId);
}
