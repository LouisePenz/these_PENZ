#include "Solution.h"

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <numeric>

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
}  // namespace


const Solution::NodeRef Solution::kArtificialDepot{-1, -1};

Solution::Solution(const Instance& instance) : instance_(&instance) {
    routes_.reserve(instance.nbVehicles());
    for (int v = 0; v < instance.nbVehicles(); ++v) {
        // Every vehicle departs the depot at time 0 and must be back by the horizon.
        routes_.emplace_back(v, instance.capacity(), PointRegistry::depot, 0, instance.horizonTime());
    }
    unassignedRequests_.reserve(instance.nbRequests());
    for (int r = 0; r < instance.nbRequests(); ++r) unassignedRequests_.push_back(r);
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

bool Solution::findRideTimeLink(int requestId, const std::vector<int>& affectedRoutes, RideTimeLink& outLink) const {
    // 2-vehicle case: looked up directly, not re-derived from route roles.
    for (const Transfer& t : transfers_) {
        if (t.requestId == requestId) {
            outLink = {{t.routeBefore, t.posPickup},
                       {t.routeAfter, t.posDelivery},
                       instance_->request(requestId).maxRideTime()};
            return true;
        }
    }

    // Otherwise: {Pickup, Delivery} (direct), or {StopAfter, Delivery} / {Pickup,
    // StopBefore} (walk on one side -- the stop plays the role of the missing end,
    // with its own derived window, already stored in its RouteStop).
    NodeRef pickupNode{-2, -2}, deliveryNode{-2, -2}, stopNode{-2, -2};
    bool hasPickup = false, hasDelivery = false, hasStop = false;
    StopRole stopRole = StopRole::StopBefore;

    for (int r : affectedRoutes) {
        const Route& route = routes_[r];
        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            if (s.requestId != requestId) continue;
            if (s.role == StopRole::Pickup) {
                pickupNode = {r, i};
                hasPickup = true;
            } else if (s.role == StopRole::Delivery) {
                deliveryNode = {r, i};
                hasDelivery = true;
            } else {
                stopNode = {r, i};
                hasStop = true;
                stopRole = s.role;
            }
        }
    }

    if (hasPickup && hasDelivery && !hasStop) {
        outLink = {pickupNode, deliveryNode, instance_->request(requestId).maxRideTime()};
        return true;
    }
    if (hasStop && hasDelivery && stopRole == StopRole::StopAfter &&
        instance_->request(requestId).walkAccess().available()) {
        outLink = {stopNode, deliveryNode, instance_->maxRideTimeLegWithWalk(requestId)};
        return true;
    }
    if (hasPickup && hasStop && stopRole == StopRole::StopBefore &&
        instance_->request(requestId).walkAccess().available()) {
        outLink = {pickupNode, stopNode, instance_->maxRideTimeLegWithWalk(requestId)};
        return true;
    }
    return false;
}

bool Solution::propagateForwardDarp(const std::vector<int>& affectedRoutes,
                                     const std::vector<std::pair<int, RideTimeLink>>& overrides) {
    const int H = instance_->horizonTime();
    std::vector<Arc> arcs;
    std::unordered_map<NodeRef, int, NodeRefHash> value;
    value[kArtificialDepot] = H;

    std::unordered_set<int> requestsSeen;

    // Route chain arcs: value(succ) >= value(pred) + driveTime + serviceTime(pred).
    for (int r : affectedRoutes) {
        const Route& route = routes_[r];
        NodeRef depotNode{r, -1};
        value[depotNode] = 0;

        NodeRef prev = depotNode;
        int prevPointId = route.depotPointId();
        int prevServiceTime = 0;
        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            NodeRef cur{r, i};
            arcs.push_back({cur, prev, -instance_->travelTimes().driveTime(prevPointId, s.pointId) - prevServiceTime});
            prev = cur;
            prevPointId = s.pointId;
            prevServiceTime = s.serviceTime;
            requestsSeen.insert(s.requestId);
        }
    }

    // Transfer arcs (2-vehicle case only): value(stopAfter) >= value(stopBefore)
    // + ptTravelTime + serviceTime(stopBefore). Only added when both ends are
    // in the affected set -- otherwise this Transfer isn't part of this call.
    for (const Transfer& t : transfers_) {
        bool beforeIn = std::find(affectedRoutes.begin(), affectedRoutes.end(), t.routeBefore) != affectedRoutes.end();
        bool afterIn = std::find(affectedRoutes.begin(), affectedRoutes.end(), t.routeAfter) != affectedRoutes.end();
        if (!beforeIn || !afterIn) continue;

        NodeRef stopBeforeNode{t.routeBefore, t.posStopBefore};
        NodeRef stopAfterNode{t.routeAfter, t.posStopAfter};
        int stopBeforeServiceTime = routes_[t.routeBefore].stopAt(t.posStopBefore).serviceTime;
        arcs.push_back({stopAfterNode, stopBeforeNode, -t.ptTravelTime - stopBeforeServiceTime});
    }

    // Per-request window + maximum-ride-time arcs, anchored to the artificial depot.
    for (int reqId : requestsSeen) {
        RideTimeLink link;

        auto overrideIt = std::find_if(overrides.begin(), overrides.end(),
                                        [reqId](const std::pair<int, RideTimeLink>& p) { return p.first == reqId; });
        if (overrideIt != overrides.end()) {
            link = overrideIt->second;
        } else if (!findRideTimeLink(reqId, affectedRoutes, link)) {
            continue;
        }

        const RouteStop& nearStop = stopAtNode(link.nearNode);
        const RouteStop& farStop = stopAtNode(link.farNode);

        arcs.push_back({kArtificialDepot, link.nearNode, nearStop.windowUtw - H});
        arcs.push_back({kArtificialDepot, link.farNode, farStop.windowUtw - H});
        arcs.push_back({link.nearNode, kArtificialDepot, H - nearStop.windowLtw});
        arcs.push_back({link.farNode, kArtificialDepot, H - farStop.windowLtw});
        arcs.push_back({link.nearNode, link.farNode, link.maxRideTime});
    }

    // Bellman-Ford relaxation, bounded by the number of distinct nodes actually
    // involved in this call (not the whole solution -- see the discussion).
    std::unordered_set<NodeRef, NodeRefHash> nodes;
    for (const Arc& a : arcs) {
        nodes.insert(a.from);
        nodes.insert(a.to);
    }
    int bound = std::max(1, static_cast<int>(nodes.size()) - 1);

    bool converged = false;
    for (int iteration = 0; iteration < bound && !converged; ++iteration) {
        converged = true;
        for (const Arc& a : arcs) {
            int toValue = value[a.to];
            int fromValue = value[a.from];
            if (toValue > fromValue + a.weight) {
                int newValue = toValue - a.weight;
                if (newValue > H) return false;  // infeasible: window exceeds the horizon
                value[a.from] = newValue;
                converged = false;
            }
        }
    }
    if (!converged) return false;  // positive cycle: no assignment of times satisfies every constraint

    for (int r : affectedRoutes) {
        Route& route = routes_[r];
        for (int i = 0; i < route.size(); ++i) {
            auto it = value.find(NodeRef{r, i});
            if (it != value.end()) route.stopAt(i).ltwDarp = it->second;
        }
    }
    return true;
}

bool Solution::propagateForwardDarp(){
    std::vector<int> allRoutes(nbRoutes());
    std::iota(allRoutes.begin(), allRoutes.end(), 0);
    return this->propagateForwardDarp(allRoutes);
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
