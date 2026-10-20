#pragma once

#include <vector>

#include "ParetoFront.h"
#include "Solution.h"

// The elementary insertion machinery for one pickup+delivery pair (already
// fully described by the caller -- point ids, service times, windows) into
// one route. Split in two so that a caller searching several routes (or
// several stop candidates) can merge everything into one globally-sorted
// list before ever running the expensive exact check -- see
// RequestInsertionSearch, which is why this split exists rather than a
// single per-route findInsertions.
class InsertionEvaluator {
public:
    struct InsertionResult {
        int routeIndex;
        int posPickup;
        int posDelivery;
        int cost;
    };

    // A candidate on the (cost, secondary metric) Pareto front for one route
    // -- see generateFrontCandidates.
    struct FrontCandidate {
        InsertionResult result;
        int metric;
    };

    // Enumerates every position pair in solution.route(routeIndex), filtered
    // by the local PDP pre-filter (see Route::pdpAdmitsInsertion), with its
    // real added cost. Does NOT run the expensive exact check.
    std::vector<InsertionResult> generateCandidates(const Solution& solution, int routeIndex,
                                                      const RouteStop& pickupTemplate,
                                                      const RouteStop& deliveryTemplate) const;

    // Same PDP-filtered enumeration, but returns only the (cost, metric)
    // Pareto front for this one route -- not every candidate. `metric` is
    // either the maximum downstream shift imposed on the route's existing
    // stops (useDropoffMetric = false), or the PDP-estimated arrival at the
    // second stop -- an optimistic proxy for its eventual DARP drop-off time
    // (useDropoffMetric = true). Both are available from the same PDP
    // preprocessing at no extra cost; pick whichever a caller wants to rank
    // candidates by before running the expensive exact check on the front's
    // members (see RequestInsertionSearch::findTransferInsertion, and the
    // discussion on why these two are meaningfully different, not just
    // relabelings of each other).
    std::vector<FrontCandidate> generateFrontCandidates(const Solution& solution, int routeIndex,
                                                          const RouteStop& pickupTemplate,
                                                          const RouteStop& secondTemplate,
                                                          bool useDropoffMetric) const;

    // The expensive exact check for exactly one candidate returned by
    // generateCandidates (for the same route and templates). Copies
    // `solution`, applies the candidate, and runs Solution::propagateForwardDarp
    // -- never mutates `solution` itself. overrideMaxRideTime is forwarded as
    // in Solution::propagateForwardDarp (see the discussion on why it's
    // needed for a 2-vehicle leg tested in isolation).
    bool verify(const Solution& solution, const InsertionResult& candidate, const RouteStop& pickupTemplate,
                const RouteStop& deliveryTemplate, const int* overrideMaxRideTime = nullptr) const;

    // Convenience for a single route: generateCandidates + sort by cost +
    // verify in that order, stopping at maxResults feasible ones. Multi-route
    // callers should use generateCandidates/verify directly and merge across
    // routes themselves before verifying (see RequestInsertionSearch).
    std::vector<InsertionResult> findInsertions(const Solution& solution, int routeIndex,
                                                 const RouteStop& pickupTemplate,
                                                 const RouteStop& deliveryTemplate, int maxResults,
                                                 const int* overrideMaxRideTime = nullptr) const;
};

