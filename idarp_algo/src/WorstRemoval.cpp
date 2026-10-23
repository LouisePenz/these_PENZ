#include "WorstRemoval.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "Solution.h"

namespace {
// Driving cost of `route` without the stops of `requestId` (0 if none is left):
// depot to first stop, between stops, last stop to depot.
int costWithout(const Route& route, const TravelTimeProvider& tt, int requestId) {
    int cost = 0;
    int prevPointId = route.depotPointId();
    bool any = false;
    for (const RouteStop& s : route.stops()) {
        if (s.requestId == requestId) continue;
        cost += tt.driveTime(prevPointId, s.pointId);
        prevPointId = s.pointId;
        any = true;
    }
    return any ? cost + tt.driveTime(prevPointId, route.depotPointId()) : 0;
}

int costOf(const Route& route, const TravelTimeProvider& tt) { return costWithout(route, tt, -1); }
}  // namespace

void WorstRemoval::apply(Solution& solution, int nbToRemove) {
    const Instance& instance = solution.instance();
    const TravelTimeProvider& tt = instance.travelTimes();

    // Routes serving each assigned request (two for a 2-vehicle one).
    std::vector<std::vector<int>> routesOf(instance.nbRequests());
    for (int r = 0; r < solution.nbRoutes(); ++r) {
        for (const RouteStop& s : solution.route(r).stops()) {
            std::vector<int>& routes = routesOf[s.requestId];
            if (routes.empty() || routes.back() != r) routes.push_back(r);
        }
    }
    std::vector<int> assigned;
    for (int req = 0; req < instance.nbRequests(); ++req)
        if (!routesOf[req].empty()) assigned.push_back(req);

    nbToRemove = std::min(nbToRemove, static_cast<int>(assigned.size()));
    std::vector<int> touchedRoutes;
    std::vector<int> routeCost(solution.nbRoutes());
    for (int r = 0; r < solution.nbRoutes(); ++r) routeCost[r] = costOf(solution.route(r), tt);

    for (int k = 0; k < nbToRemove; ++k) {
        // Saving of each assigned request on the current routes, largest first
        // (ties: smallest request id, for reproducibility).
        std::vector<std::pair<int, int>> savings;  // (saving, request)
        savings.reserve(assigned.size());
        for (int req : assigned) {
            int saving = 0;
            for (int r : routesOf[req]) saving += routeCost[r] - costWithout(solution.route(r), tt, req);
            savings.push_back({saving, req});
        }
        std::sort(savings.begin(), savings.end(), [](const auto& a, const auto& b) {
            return a.first != b.first ? a.first > b.first : a.second < b.second;
        });

        double y = rand() / (static_cast<double>(RAND_MAX) + 1.0);
        int rank = static_cast<int>(std::pow(y, randomness_) * savings.size());
        int requestId = savings[rank].second;

        solution.removeRequest(requestId);
        for (int r : routesOf[requestId]) {
            routeCost[r] = costOf(solution.route(r), tt);
            touchedRoutes.push_back(r);
        }
        routesOf[requestId].clear();
        assigned.erase(std::find(assigned.begin(), assigned.end(), requestId));
    }

    if (touchedRoutes.empty()) return;

    // Removing stops only relaxes constraints, so this cannot fail; it also
    // splits the sets of linked routes whose last transfer was removed.
    std::sort(touchedRoutes.begin(), touchedRoutes.end());
    touchedRoutes.erase(std::unique(touchedRoutes.begin(), touchedRoutes.end()), touchedRoutes.end());
    solution.refreshTiming(touchedRoutes);
}
