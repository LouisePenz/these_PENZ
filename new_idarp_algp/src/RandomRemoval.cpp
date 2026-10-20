#include "RandomRemoval.h"

#include <algorithm>
#include <cstdlib>

#include "Solution.h"

void RandomRemoval::apply(Solution& solution, int nbToRemove) {
    const Instance& instance = solution.instance();

    std::vector<bool> isUnassigned(instance.nbRequests(), false);
    for (int r : solution.unassignedRequests()) isUnassigned[r] = true;

    std::vector<int> assigned;
    for (int r = 0; r < instance.nbRequests(); ++r) {
        if (!isUnassigned[r]) assigned.push_back(r);
    }

    nbToRemove = std::min(nbToRemove, static_cast<int>(assigned.size()));
    std::vector<int> touchedRoutes;

    for (int k = 0; k < nbToRemove; ++k) {
        int idx = rand() % static_cast<int>(assigned.size());
        int requestId = assigned[idx];
        assigned[idx] = assigned.back();
        assigned.pop_back();

        for (int r = 0; r < solution.nbRoutes(); ++r) {
            const Route& route = solution.route(r);
            bool found = false;
            for (int i = 0; i < route.size() && !found; ++i) {
                if (route.stopAt(i).requestId == requestId) {
                    touchedRoutes.push_back(r);
                    found = true;
                }
            }
        }
        solution.removeRequest(requestId);
    }

    if (touchedRoutes.empty()) return;

    std::sort(touchedRoutes.begin(), touchedRoutes.end());
    touchedRoutes.erase(std::unique(touchedRoutes.begin(), touchedRoutes.end()), touchedRoutes.end());

    std::vector<int> affected;
    for (int r : touchedRoutes) {
        for (int dep : solution.dependentRoutes(r)) affected.push_back(dep);
    }
    std::sort(affected.begin(), affected.end());
    affected.erase(std::unique(affected.begin(), affected.end()), affected.end());

    solution.propagateForwardDarp(affected);
}
