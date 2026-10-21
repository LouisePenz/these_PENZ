#include "RegretRepair.h"

#include <limits>

#include "Solution.h"

bool RegretRepair::apply(Solution& solution, bool allowPartial) {
    while (true) {
        std::vector<int> pending = solution.unassignedRequests();
        if (pending.empty()) return true;

        bool foundAny = false;
        long long bestRegret = -1;
        RequestInsertionCandidate bestCandidate;

        for (int requestId : pending) {
            // Best insertion, and best insertion on other routes.
            auto candidates = search_.findInsertions(solution, requestId, 2, /*distinctRoutes=*/true);
            if (candidates.empty()) {
                // Inserting other requests only adds constraints (drive times
                // satisfy the triangle inequality): this one will never be
                // insertable in this repair, so a full repair is already lost.
                if (!allowPartial) return false;
                continue;
            }

            long long regret = (candidates.size() >= 2) ? static_cast<long long>(candidates[1].cost) - candidates[0].cost
                                                          : std::numeric_limits<long long>::max();
            // Highest regret; ties broken by the cheapest best insertion, then
            // by the order of unassignedRequests().
            if (!foundAny || regret > bestRegret || (regret == bestRegret && candidates[0].cost < bestCandidate.cost)) {
                foundAny = true;
                bestRegret = regret;
                bestCandidate = candidates[0];
            }
        }

        if (!foundAny) return allowPartial;
        search_.apply(solution, bestCandidate);
    }
}
