#include "RoutePool.h"

#include "Solution.h"

std::size_t RoutePool::KeyHash::operator()(const Key& key) const noexcept {
    std::uint64_t h = 1469598103934665603ULL;
    for (std::uint64_t v : key) {
        h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return static_cast<std::size_t>(h);
}

RoutePool::Key RoutePool::keyOf(const Solution& solution, int routeIndex, std::vector<char>& transferLeg) {
    const Route& route = solution.route(routeIndex);
    transferLeg.assign(route.size(), 0);
    for (const Transfer& t : solution.transfers()) {
        if (t.routeBefore == routeIndex) transferLeg[t.posPickup] = transferLeg[t.posStopBefore] = 1;
        if (t.routeAfter == routeIndex) transferLeg[t.posStopAfter] = transferLeg[t.posDelivery] = 1;
    }
    Key key(route.size());
    for (int i = 0; i < route.size(); ++i) {
        const RouteStop& s = route.stopAt(i);
        key[i] = (static_cast<std::uint64_t>(s.requestId) << 32) | (static_cast<std::uint64_t>(s.pointId) << 3) |
                 (static_cast<std::uint64_t>(s.role) << 1) | static_cast<std::uint64_t>(transferLeg[i]);
    }
    return key;
}

void RoutePool::add(const Solution& solution) {
    std::vector<char> transferLeg;
    for (int r = 0; r < solution.nbRoutes(); ++r) {
        const Route& route = solution.route(r);
        if (route.empty()) continue;
        Key key = keyOf(solution, r, transferLeg);
        if (index_.count(key)) continue;

        PoolRoute poolRoute;
        poolRoute.stops = route.stops();
        poolRoute.transferLeg = transferLeg;
        const TravelTimeProvider& tt = solution.instance().travelTimes();
        int prevPointId = route.depotPointId();
        for (const RouteStop& s : route.stops()) {
            poolRoute.cost += tt.driveTime(prevPointId, s.pointId);
            prevPointId = s.pointId;
        }
        poolRoute.cost += tt.driveTime(prevPointId, route.depotPointId());

        index_.emplace(std::move(key), size());
        routes_.push_back(std::move(poolRoute));
    }
}

int RoutePool::find(const Solution& solution, int routeIndex) const {
    std::vector<char> transferLeg;
    auto it = index_.find(keyOf(solution, routeIndex, transferLeg));
    return it == index_.end() ? -1 : it->second;
}
