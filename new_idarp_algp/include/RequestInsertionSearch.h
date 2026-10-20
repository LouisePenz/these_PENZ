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
// unassigned request: every candidate of every pattern admitted by the O(1)
// test (InsertionEvaluator) goes into ONE list, sorted once (see
// candidateLess), then the exact STP check runs in that order until enough
// feasible ones are found -- so no STP check is ever spent on a candidate
// costlier than the ones finally returned.
class RequestInsertionSearch {
public:
    // Returns up to maxResults feasible ways to insert `requestId`, best first
    // (see isBetter). Does not mutate `solution` -- see apply() to actually
    // commit a chosen candidate. With `onlyTouching`, only the candidates
    // using at least one of the marked routes (indexed by route) are considered.
    std::vector<RequestInsertionCandidate> findInsertions(const Solution& solution, int requestId, int maxResults,
                                                            const std::vector<bool>* onlyTouching = nullptr) const;

    // Commits a candidate previously returned by findInsertions(): inserts
    // its stop(s) into the real solution, adds a Transfer if needed, marks
    // the request assigned, and refreshes the timing data of the routes involved.
    //
    // Returns the routes (marked, indexed by route) whose insertion candidates
    // may have changed: every route linked, after this insertion, to a route
    // the candidate uses. Any candidate of another request using none of them
    // keeps exactly the same cost and feasibility.
    std::vector<bool> apply(Solution& solution, const RequestInsertionCandidate& candidate) const;

    // Incremental findInsertions(): `best` holds the result of
    // findInsertions(solution, requestId, maxResults) BEFORE an apply() that
    // returned `affected`; it becomes the result for the current solution.
    // - none of `best` touches `affected`: they are all still valid, and only
    //   the candidates touching `affected` can do better -- only those are searched;
    // - one of them does: full search;
    // - `best` is empty: it stays empty, no search at all. Inserting a request
    //   only adds constraints (drive times satisfy the triangle inequality),
    //   so a request that cannot be inserted never becomes insertable.
    void updateInsertions(const Solution& solution, int requestId, int maxResults, const std::vector<bool>& affected,
                          std::vector<RequestInsertionCandidate>& best) const;

    // The order of findInsertions(): cost, then pattern (direct, walk,
    // 2-vehicle), then routes, positions and stops -- a strict total order,
    // so equal-cost candidates always come out the same way.
    static bool isBetter(const RequestInsertionCandidate& a, const RequestInsertionCandidate& b);

    // True if `candidate` uses one of the marked routes.
    static bool touches(const RequestInsertionCandidate& candidate, const std::vector<bool>& routes);

private:
    // Each appends to `out` every candidate of its pattern admitted by the
    // O(1) test -- none of them checked by the STP yet -- restricted as in
    // findInsertions() when onlyTouching is given.
    void collectDirect(const Solution& solution, int requestId, const std::vector<bool>* onlyTouching,
                       std::vector<RequestInsertionCandidate>& out) const;
    void collectWalk(const Solution& solution, int requestId, const std::vector<bool>* onlyTouching,
                     std::vector<RequestInsertionCandidate>& out) const;
    // Exhaustive: every (first leg, last leg) combination -- every valid pair
    // of PT stops, every pair of distinct routes, every position admitted for
    // each leg -- that passes the O(1) test of the whole request
    // (InsertionEvaluator::admitsTransfer).
    void collectTransfer(const Solution& solution, int requestId, const std::vector<bool>* onlyTouching,
                         std::vector<RequestInsertionCandidate>& out) const;

    // Exact check of any candidate: its stop(s), and Transfer if any, are
    // inserted in a copy of `solution`, then capacity and the STP of the
    // routes involved (with the request's true maximum ride time) are checked.
    bool verifyCandidate(const Solution& solution, const RequestInsertionCandidate& candidate) const;

    InsertionEvaluator evaluator_;
};
