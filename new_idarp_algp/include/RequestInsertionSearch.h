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
// unassigned request. For each pattern: every candidate admitted by the O(1)
// test (InsertionEvaluator), sorted by cost, then the exact STP check in that
// order until enough feasible ones are found.
class RequestInsertionSearch {
public:
    // Returns up to maxResults feasible ways to insert `requestId`, sorted by
    // total cost ascending. Does not mutate `solution` -- see apply() to
    // actually commit a chosen candidate.
    std::vector<RequestInsertionCandidate> findInsertions(const Solution& solution, int requestId,
                                                            int maxResults) const;

    // Commits a candidate previously returned by findInsertions(): inserts
    // its stop(s) into the real solution, adds a Transfer if needed, marks
    // the request assigned, and refreshes the timing data of the routes involved.
    void apply(Solution& solution, const RequestInsertionCandidate& candidate) const;

private:
    std::vector<RequestInsertionCandidate> findDirectInsertions(const Solution& solution, int requestId,
                                                                  int maxResults) const;
    std::vector<RequestInsertionCandidate> findWalkInsertions(const Solution& solution, int requestId,
                                                                int maxResults) const;

    // Exhaustive: every (first leg, last leg) combination -- every valid pair
    // of PT stops, every pair of distinct routes, every position admitted for
    // each leg -- that passes the O(1) test of the whole request
    // (InsertionEvaluator::admitsTransfer), sorted by total cost, then the
    // exact STP check of the whole request in that order.
    std::vector<RequestInsertionCandidate> findTransferInsertions(const Solution& solution, int requestId,
                                                                    int maxResults) const;

    // Exact check of a 2-vehicle candidate: both legs and the Transfer are
    // inserted in a copy of `solution`, then capacity and the STP of the
    // whole request (its true maximum ride time) are checked.
    bool verifyTransfer(const Solution& solution, const RequestInsertionCandidate& candidate) const;

    InsertionEvaluator evaluator_;
};
