#include "RelatedRemoval.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "Solution.h"

void RelatedRemoval::apply(Solution& solution, int nbToRemove) {
    const Instance& instance = solution.instance();

    std::vector<bool> isAssigned(instance.nbRequests(), true);
    for (int r : solution.unassignedRequests()) isAssigned[r] = false;
    int nbAssigned = instance.nbRequests() - static_cast<int>(solution.unassignedRequests().size());
    nbToRemove = std::min(nbToRemove, nbAssigned);
    if (nbToRemove <= 0) return;

    // First request: uniformly among the assigned ones.
    std::vector<bool> selected(instance.nbRequests(), false);
    std::vector<int> selection;
    int first = rand() % nbAssigned;
    for (int r = 0; r < instance.nbRequests(); ++r) {
        if (isAssigned[r] && first-- == 0) {
            selected[r] = true;
            selection.push_back(r);
            break;
        }
    }

    while (static_cast<int>(selection.size()) < nbToRemove) {
        int reference = selection[rand() % selection.size()];
        const std::vector<int>& related = (measure_ == Measure::AnyEnds)
                                              ? instance.relatedRequestsWithTransfer(reference)
                                              : instance.relatedRequests(reference);
        // L: the assigned requests not selected yet, most related first.
        int sizeL = nbAssigned - static_cast<int>(selection.size());
        double y = rand() / (static_cast<double>(RAND_MAX) + 1.0);
        int rank = static_cast<int>(std::pow(y, randomness_) * sizeL);
        for (int r : related) {
            if (!isAssigned[r] || selected[r]) continue;
            if (rank-- == 0) {
                selected[r] = true;
                selection.push_back(r);
                break;
            }
        }
    }

    std::vector<int> touchedRoutes;
    for (int r = 0; r < solution.nbRoutes(); ++r) {
        const Route& route = solution.route(r);
        for (int i = 0; i < route.size(); ++i) {
            if (selected[route.stopAt(i).requestId]) {
                touchedRoutes.push_back(r);
                break;
            }
        }
    }
    for (int requestId : selection) solution.removeRequest(requestId);

    // Removing stops only relaxes constraints, so this cannot fail; it also
    // splits the sets of linked routes whose last transfer was removed.
    solution.refreshTiming(touchedRoutes);
}
