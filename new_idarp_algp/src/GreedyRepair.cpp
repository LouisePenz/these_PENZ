#include "GreedyRepair.h"

#include <unordered_map>

#include "Solution.h"

bool GreedyRepair::apply(Solution& solution, bool allowPartial) {
    // Best insertion of every pending request, kept up to date after each
    // insertion by RequestInsertionSearch::updateInsertions (only the
    // candidates touching the routes it changed are searched again).
    std::unordered_map<int, std::vector<RequestInsertionCandidate>> best;
    for (int requestId : solution.unassignedRequests()) {
        best[requestId] = search_.findInsertions(solution, requestId, 1);
        // Cannot be inserted now, so never in this repair: no need to go on.
        if (best[requestId].empty() && !allowPartial) return false;
    }

    while (true) {
        // Cheapest insertion; ties: first request in unassignedRequests() order.
        const RequestInsertionCandidate* chosen = nullptr;
        for (int requestId : solution.unassignedRequests()) {
            const auto& candidates = best[requestId];
            if (!candidates.empty() && (chosen == nullptr || candidates[0].cost < chosen->cost))
                chosen = &candidates[0];
        }
        if (chosen == nullptr) return solution.unassignedRequests().empty() || allowPartial;

        RequestInsertionCandidate inserted = *chosen;
        best.erase(inserted.requestId);
        std::vector<bool> affected = search_.apply(solution, inserted);

        for (auto& [requestId, candidates] : best) {
            search_.updateInsertions(solution, requestId, 1, affected, candidates);
            if (candidates.empty() && !allowPartial) return false;
        }
    }
}
