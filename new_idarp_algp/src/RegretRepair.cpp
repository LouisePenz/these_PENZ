#include "RegretRepair.h"

#include <limits>

#include "Solution.h"
#include <iostream>

bool RegretRepair::apply(Solution& solution, bool allowPartial) {
    while (true) {
        std::vector<int> pending = solution.unassignedRequests();
        if (pending.empty()) return true;

        bool foundAny = false;
        long long bestRegret = -1;
        RequestInsertionCandidate bestCandidate;

        for (int requestId : pending) {
            auto candidates = search_.findInsertions(solution, requestId, 2);
            if (candidates.empty()) continue;
            foundAny = true;

            long long regret = (candidates.size() >= 2) ? static_cast<long long>(candidates[1].cost) - candidates[0].cost
                                                          : std::numeric_limits<long long>::max();
            if (regret > bestRegret) {
                bestRegret = regret;
                bestCandidate = candidates[0];
            }
        }

        if (!foundAny) return allowPartial;
        search_.apply(solution, bestCandidate);
    }
}
