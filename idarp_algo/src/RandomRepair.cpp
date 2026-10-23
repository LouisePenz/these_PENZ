#include "RandomRepair.h"

#include <cstdlib>
#include <utility>

#include "Solution.h"

bool RandomRepair::apply(Solution& solution, bool allowPartial) {
    std::vector<int> pending = solution.unassignedRequests();  // snapshot: apply() mutates it

    // Fisher-Yates shuffle with rand(), consistent with RandomRemoval.
    for (int i = static_cast<int>(pending.size()) - 1; i > 0; --i) {
        int j = rand() % (i + 1);
        std::swap(pending[i], pending[j]);
    }

    for (int requestId : pending) {
        //std::cout << "request: " << requestId << "\n";
        auto candidates = search_.findInsertions(solution, requestId, 1);
        if (candidates.empty()) {
            if (!allowPartial) return false;
            continue;
        }
        search_.apply(solution, candidates[0]);
    }
    return true;
}
