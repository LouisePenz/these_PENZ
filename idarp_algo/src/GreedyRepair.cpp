#include "GreedyRepair.h"

#include "Solution.h"

bool GreedyRepair::apply(Solution& solution, bool allowPartial) {
    while (true) {
        std::vector<int> pending = solution.unassignedRequests();
        if (pending.empty()) return true;

        bool foundAny = false;
        int bestCost = 0;
        RequestInsertionCandidate bestCandidate;

        for (int requestId : pending) {
            auto candidates = search_.findInsertions(solution, requestId, 1);
            if (candidates.empty()) {
                // Inserting other requests only adds constraints (drive times
                // satisfy the triangle inequality): this one will never be
                // insertable in this repair, so a full repair is already lost.
                if (!allowPartial) return false;
                continue;
            }
            if (!foundAny || candidates[0].cost < bestCost) {
                foundAny = true;
                bestCost = candidates[0].cost;
                bestCandidate = candidates[0];
            }
        }

        if (!foundAny) return allowPartial;  // some requests stay unassigned
        search_.apply(solution, bestCandidate);
    }
}
