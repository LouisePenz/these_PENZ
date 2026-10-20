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
// unassigned request.
//
// NOTE on the transfer pattern: the first leg is searched via a per-route
// Pareto front (cost, secondary metric) merged across routes -- see
// findTransferInsertion for the choice of metric. Every feasible front member
// is then paired with its own best second leg, and the overall cheapest
// combination is kept. The second leg itself is still a single best search,
// not a front -- see the discussion on why this was scoped to the first leg
// for now.
class RequestInsertionSearch {
public:
    // Returns up to maxResults feasible ways to insert `requestId`, sorted by
    // total cost ascending. Does not mutate `solution` -- see apply() to
    // actually commit a chosen candidate. useDropoffMetric is forwarded to
    // the transfer pattern's first-leg front (see findTransferInsertion).
    std::vector<RequestInsertionCandidate> findInsertions(const Solution& solution, int requestId, int maxResults,
                                                            bool useDropoffMetric = false) const;

    // Commits a candidate previously returned by findInsertions(): inserts
    // its stop(s) into the real solution, adds a Transfer if needed, marks
    // the request assigned, and re-propagates the affected routes.
    void apply(Solution& solution, const RequestInsertionCandidate& candidate) const;

    // Diagnostic counters for the transfer pattern's first-leg front: how many
    // candidates the (merged) front contained and therefore got verified, and
    // how many of those turned out feasible for the first leg alone. Cheap and
    // permanent (unlike RequestInsertionSearchComplete.*), useful for tuning.
    struct TransferFrontStats {
        int task1FrontSize = 0;
        int task1FeasibleFound = 0;
    };

    // Returns an empty vector if no feasible transfer was found. useDropoffMetric
    // selects the second dimension of the first-leg Pareto front: false = max
    // downstream shift, true = PDP-estimated drop-off time (see the
    // discussion on why these are genuinely different, not equivalent, and
    // why both are exposed for empirical comparison rather than picking one).
    //
    // TEMPORARILY public so RequestInsertionSearchComplete.* (a throwaway
    // comparison tool) can call it directly -- move back under `private:`
    // once those files are deleted.
    std::vector<RequestInsertionCandidate> findTransferInsertion(const Solution& solution, int requestId,
                                                                    bool useDropoffMetric,
                                                                    TransferFrontStats* outStats = nullptr) const;

private:
    std::vector<RequestInsertionCandidate> findDirectInsertions(const Solution& solution, int requestId,
                                                                  int maxResults) const;
    std::vector<RequestInsertionCandidate> findWalkInsertions(const Solution& solution, int requestId,
                                                                int maxResults) const;

    struct SecondLegResult {
        bool found = false;
        InsertionEvaluator::InsertionResult result;
        RouteStop stopAfter;
        int ptTravelTime = 0;
    };
    // Cheapest feasible second leg (stopAfter -> delivery), merged across
    // every exit stop and every route except the first leg's route, tried in
    // global cost order. A candidate is accepted only if the WHOLE request is
    // feasible: the first leg (already chosen) and the candidate second leg
    // are both inserted, the real Transfer is added, and the exact STP check
    // is run with the request's true maxRideTime over the whole
    // pickup -> delivery chain. There is deliberately no separate per-leg
    // test: validating each leg alone against an optimistic per-leg bound let
    // infeasible combinations through (the split of the ride-time budget
    // between the two vehicles is a real degree of freedom).
    SecondLegResult findBestSecondLeg(const Solution& solution, int requestId, const RouteStop& pickup,
                                        const RouteStop& stopBefore, const RouteStop& delivery, int entryStop,
                                        const InsertionEvaluator::InsertionResult& task1,
                                        int achievedDropoff) const;

    InsertionEvaluator evaluator_;
};