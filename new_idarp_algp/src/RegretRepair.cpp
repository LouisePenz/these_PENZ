#include "RegretRepair.h"

#include <limits>
#include <unordered_map>

#include "Solution.h"

bool RegretRepair::apply(Solution& solution, bool allowPartial) {
    // Two best insertions of every pending request, kept up to date after
    // each insertion by RequestInsertionSearch::updateInsertions (only the
    // candidates touching the routes it changed are searched again).
    std::unordered_map<int, std::vector<RequestInsertionCandidate>> best;
    for (int requestId : solution.unassignedRequests()) {
        best[requestId] = search_.findInsertions(solution, requestId, 2);
        // Cannot be inserted now, so never in this repair: no need to go on.
        if (best[requestId].empty() && !allowPartial) return false;
    }

    while (true) {
        // Highest regret; ties: first request in unassignedRequests() order.
        const RequestInsertionCandidate* chosen = nullptr;
        long long bestRegret = -1;
        for (int requestId : solution.unassignedRequests()) {
            const auto& candidates = best[requestId];
            if (candidates.empty()) continue;
            long long regret = (candidates.size() >= 2)
                                   ? static_cast<long long>(candidates[1].cost) - candidates[0].cost
                                   : std::numeric_limits<long long>::max();
            if (regret > bestRegret) {
                bestRegret = regret;
                chosen = &candidates[0];
            }
        }
        if (chosen == nullptr) return solution.unassignedRequests().empty() || allowPartial;

        RequestInsertionCandidate inserted = *chosen;
        best.erase(inserted.requestId);
        std::vector<bool> affected = search_.apply(solution, inserted);

        for (auto& [requestId, candidates] : best) {
            search_.updateInsertions(solution, requestId, 2, affected, candidates);
            if (candidates.empty() && !allowPartial) return false;
        }
    }
}
