#pragma once

#include <limits>
#include <vector>

#include "LinkedRoutesTiming.h"
#include "Solution.h"

// The elementary insertion machinery for one pair of stops (already fully
// described by the caller -- point ids, service times, windows) into one
// route. Split in two so that a caller searching several routes (or several
// stop candidates) can merge everything into one globally-sorted list before
// ever running the expensive exact check -- see RequestInsertionSearch.
//
// The cheap pre-filter is the constant-time test of Masson, Lehuédé & Péton
// (2013) on the solution's LinkedRoutesTiming: it sees the delays an insertion
// propagates to OTHER routes through transfers, and the maximum ride times
// through the exact DARP windows -- but it remains a necessary condition only.
class InsertionEvaluator {
public:
    struct InsertionResult {
        int routeIndex;
        int posPickup;
        int posDelivery;
        int cost;
    };

    // For a leg whose two stops are not linked by a maximum ride time of their own.
    static constexpr int kNoRideTimeLimit = std::numeric_limits<int>::max() / 4;

    // Outcome of the O(1) test for one leg: `first` inserted at posFirst and
    // `second` at posSecond of one route (same convention as InsertionResult:
    // positions in the route BEFORE insertion, posSecond == posFirst meaning
    // right after `first`).
    struct LegEstimate {
        struct Shift {
            int node;   // in `timing`
            int delta;  // postponement of that existing stop
        };
        bool admitted = false;
        int firstTime = 0;    // earliest service time of `first` after insertion
        int secondTime = 0;   // same for `second`
        int firstLatest = 0;  // upper bound on the latest service time of `first`
        const LinkedRoutesTiming* timing = nullptr;
        Shift shifts[2];  // at the existing stop(s) right after the inserted ones
        int nbShifts = 0;
    };

    // Algorithms 1 and 2 of the paper, plus: the latest-time bounds of the two
    // new stops, and `maxRideTime` between them. `upstream`: shifts already
    // caused by the other leg of the same request (Algorithm 3); firstMinTime:
    // extra lower bound on the service time of `first`.
    LegEstimate estimateLeg(const Solution& solution, int routeIndex, int posFirst, int posSecond,
                            const RouteStop& first, const RouteStop& second, int maxRideTime,
                            const LegEstimate* upstream = nullptr, int firstMinTime = 0) const;

    // Algorithm 3: both legs of a 2-vehicle request at once -- pickup and
    // stopBefore in leg1's route, stopAfter and delivery in leg2's -- with the
    // cycle test of Proposition 4 and the request's whole maximum ride time.
    bool admitsTransfer(const Solution& solution, const InsertionResult& leg1, const RouteStop& pickup,
                        const RouteStop& stopBefore, const InsertionResult& leg2, const RouteStop& stopAfter,
                        const RouteStop& delivery, int ptTravelTime, int maxRideTime) const;

    // Every position pair in solution.route(routeIndex) admitted by
    // estimateLeg, with its real added cost. Does NOT run the exact check.
    std::vector<InsertionResult> generateCandidates(const Solution& solution, int routeIndex,
                                                      const RouteStop& pickupTemplate,
                                                      const RouteStop& deliveryTemplate, int maxRideTime) const;

private:
    // estimateLeg when the second stop goes further in the route (posSecond >
    // posFirst), split in two so that the first stop's part -- which only
    // depends on posFirst -- is computed once for every posSecond.
    struct FirstStop {
        bool admitted = false;
        int routeIndex = -1;
        int time = 0;       // earliest service time of the first stop
        int latest = 0;     // upper bound on its latest service time
        int afterNode = 0;  // node (in `timing`) of the existing stop right after it
        int shift = 0;      // postponement it imposes on that stop
        const LinkedRoutesTiming* timing = nullptr;
    };
    FirstStop placeFirst(const Solution& solution, int routeIndex, int posFirst, const RouteStop& first,
                         const LegEstimate* upstream = nullptr, int firstMinTime = 0) const;
    LegEstimate placeSecond(const Solution& solution, const FirstStop& first, int posSecond, const RouteStop& second,
                            int maxRideTime, const LegEstimate* upstream = nullptr) const;
};
