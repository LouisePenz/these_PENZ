#pragma once

#include <vector>

#include "InsertionEvaluator.h"
#include "Solution.h"

// One vehicle-served leg of a candidate: which route, the two RouteStop
// templates (already fully filled in), and the positions InsertionEvaluator
// found for them (same convention as InsertionEvaluator::InsertionResult).
struct LegPlacement {
    int routeIndex = -1;
    RouteStop firstStop;
    RouteStop secondStop;
    int posFirst = -1;
    int posSecond = -1;
};

// A full candidate way to insert one request: either a single leg (direct
// request, or one vehicle leg + a walk on the other side), or two legs linked
// by a Transfer (2-vehicle PT). Cost is the total added driving cost.
struct RequestInsertionCandidate {
    int requestId = -1;
    int cost = 0;
    bool isTransfer = false;

    LegPlacement leg1;  // direct/walk: the only leg. transfer: pickup -> stopBefore.
    LegPlacement leg2;  // transfer only: stopAfter -> delivery.
    int ptTravelTime = 0;
};

// Orchestrates InsertionEvaluator across every route and every pattern (direct,
// walk, 2-vehicle transfer) to find the best way(s) to insert one currently
// unassigned request. Candidates admitted by the O(1) test (InsertionEvaluator)
// are checked by the exact STP in cost order (candidateLess) until enough
// feasible ones are found: first the direct and walk ones, then only the
// 2-vehicle ones that can still beat them (see findInsertions) -- the latter
// being by far the most numerous, most of them are never even generated.
class RequestInsertionSearch {
public:
    // Returns up to maxResults feasible ways to insert `requestId`, sorted by
    // total cost ascending. Does not mutate `solution` -- see apply() to
    // actually commit a chosen candidate.
    //
    // distinctRoutes: the results must use pairwise different routes -- the
    // pair (first leg's route, second leg's route), a direct or walk candidate
    // counting as (its route, none). E.g. the best insertion and the best one
    // in another route, as the regret heuristic needs (Ropke & Pisinger 2006).
    std::vector<RequestInsertionCandidate> findInsertions(const Solution& solution, int requestId, int maxResults,
                                                            bool distinctRoutes = false) const;

    // Commits a candidate previously returned by findInsertions(): inserts
    // its stop(s) into the real solution, adds a Transfer if needed, marks
    // the request assigned, and refreshes the timing data of the routes involved.
    void apply(Solution& solution, const RequestInsertionCandidate& candidate) const;

private:
    // Each appends to `out` every candidate of its pattern admitted by the
    // O(1) test -- none of them checked by the STP yet.
    void collectDirect(const Solution& solution, int requestId, std::vector<RequestInsertionCandidate>& out) const;
    void collectWalk(const Solution& solution, int requestId, std::vector<RequestInsertionCandidate>& out) const;
    // Exhaustive below costBound: every (first leg, last leg) combination --
    // every valid pair of PT stops, every pair of distinct routes, every
    // position admitted for each leg -- costing strictly less than costBound
    // and passing the O(1) test of the whole request
    // (InsertionEvaluator::admitsTransfer).
    void collectTransfer(const Solution& solution, int requestId, int costBound,
                         std::vector<RequestInsertionCandidate>& out) const;

    // Checks `candidates` with verifyCandidate in candidateLess order until
    // maxResults feasible ones are found, and returns them in that order.
    // With distinctRoutes, a candidate on the same routes as one already
    // found is skipped without being checked. Consumes `candidates` (left in
    // an unspecified state).
    std::vector<RequestInsertionCandidate> verifyInOrder(const Solution& solution,
                                                          std::vector<RequestInsertionCandidate>& candidates,
                                                          int maxResults, bool distinctRoutes) const;

    // Exact check of any candidate: its stop(s), and Transfer if any, are
    // inserted in a copy of `solution`, then capacity and the STP of the
    // routes involved (with the request's true maximum ride time) are checked.
    bool verifyCandidate(const Solution& solution, const RequestInsertionCandidate& candidate) const;

    InsertionEvaluator evaluator_;
};
