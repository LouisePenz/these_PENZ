#pragma once

#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Instance.h"
#include "Route.h"
#include "Transfer.h"

class LinkedRoutesTiming;
class RouteDarpTiming;

// The state currently under construction/repair by the LNS: one Route per
// vehicle, the Transfer links between routes for 2-vehicle PT requests, and
// the requests not yet assigned to any route.
class Solution {
public:
    explicit Solution(const Instance& instance);

    const Instance& instance() const noexcept { return *instance_; }

    int nbRoutes() const noexcept { return static_cast<int>(routes_.size()); }
    Route& route(int index) { return routes_[index]; }
    const Route& route(int index) const { return routes_[index]; }
    std::vector<Route>& routes() noexcept { return routes_; }
    const std::vector<Route>& routes() const noexcept { return routes_; }

    const std::vector<Transfer>& transfers() const noexcept { return transfers_; }
    void addTransfer(Transfer transfer) { transfers_.push_back(transfer); }
    // Removes the Transfer belonging to this request, if any.
    void removeTransfer(int requestId);

    const std::vector<int>& unassignedRequests() const noexcept { return unassignedRequests_; }
    void markUnassigned(int requestId) { unassignedRequests_.push_back(requestId); }
    void markAssigned(int requestId);

    // Every route index reachable from `start` by following Transfer links --
    // the set of routes that must be propagated together in a single call to
    // propagateForwardDarp (they share constraints through their transfers).
    std::vector<int> dependentRoutes(int start) const;

    // The exact DARP feasibility test (Simple Temporal Problem -- Cordeau /
    // Masson & Lehuédé / Molenbruch), restricted to the routes in
    // `affectedRoutes`. Callers must pass an already-closed set (typically
    // the result of dependentRoutes()): a request split across a route that
    // isn't included won't be checked, silently.
    //
    // On success, updates RouteStop::ltwDarp (earliest feasible time) for
    // every stop in `affectedRoutes` and returns true. On failure, returns
    // false and leaves every ltwDarp untouched.
    //
    // warmStart: start the relaxation from the stops' current ltwDarp instead
    // of 0 -- same result, fewer passes. ONLY valid when every current ltwDarp
    // is a lower bound of the new earliest schedule, i.e. when this solution
    // is a solution whose ltwDarp were exact (refreshTiming keeps them so)
    // with requests ADDED only (new stops carry ltwDarp = 0). After a
    // removal, windows widen, earliest times can decrease, and a warm start
    // would return wrong (too late) times -- never use it there.
    bool propagateForwardDarp(const std::vector<int>& affectedRoutes, bool warmStart = false);

    bool propagateForwardDarp();

    // Same constraint graph, opposite direction: on success, updates
    // RouteStop::utwDarp (latest feasible time) for every stop in
    // `affectedRoutes`. Together with ltwDarp, [ltwDarp; utwDarp] is the
    // exact interval of times each stop can take in SOME feasible schedule of
    // the current solution -- narrower than the PDP windows, since it accounts
    // for every route linked by a transfer and for the maximum ride times.
    bool propagateBackwardDarp(const std::vector<int>& affectedRoutes);
    bool propagateBackwardDarp();

    // Timing data of the set of linked routes containing `route`, for the O(1)
    // insertion test (see LinkedRoutesTiming). Up to date as long as every
    // change to the routes goes through refreshTiming() afterwards.
    const LinkedRoutesTiming& timing(int route) const;
    // The route seen as a route of the standard DARP, for the exact O(1)
    // insertion test of Gschwind & Drexl (see RouteDarpTiming). Up to date
    // under the same condition as timing().
    const RouteDarpTiming& routeTiming(int route) const;

    // To call after any change to the routes or transfers: on every set of
    // linked routes containing one of `changedRoutes` -- as they are NOW, so a
    // removed transfer that split a set, or an added one that merged two, is
    // handled -- recomputes the loads, re-runs the exact STP forward and
    // backward (ltwDarp/utwDarp), and rebuilds the timing data (both
    // LinkedRoutesTiming and RouteDarpTiming). Returns false
    // if one of those sets has no feasible schedule.
    bool refreshTiming(const std::vector<int>& changedRoutes);

    // Human-readable dump of the current state (routes, stops, transfers,
    // unassigned requests) -- deliberately not the request/instance data
    // already printed by Instance::print().
    void print() const;

    // Writes the solution in a text format meant for other tools (main_LNS -sol), with the times of
    // `schedule` (route x position, e.g. from ridingTimeHeuristic). Times in the instance's unit.
    //   routes <nbRoutes>
    //   route <r> <nbStops>, then one line per stop: <P|D|SB|SA> <requestId> <pointId> <time>
    //     (P pickup, D delivery, SB leaves the vehicle for PT, SA boards it from PT; depot implicit)
    //   requests <nbRequests>, then one line per request:
    //     <id> direct <route>
    //     <id> walk_pickup <route> <nearestStop> <walkTime> <leave> <ptDep> <ptArr> <stop>
    //     <id> walk_delivery <route> <stop> <ptDep> <ptArr> <nearestStop> <walkTime> <arrival>
    //     <id> transfer <routeBefore> <routeAfter> <stopBefore> <stopAfter> <ptDep> <ptArr>
    //     <id> unassigned
    // ptDep / ptArr: the tram run with the timetable (-tram); with the PT matrix, the ride ending
    // or starting at the vehicle's stop (average waiting included in its duration).
    void write(std::ostream& out, const std::vector<std::vector<int>>& schedule) const;
    // Writes `header` (one "key value" line each), the objective values (objective_type, route_time,
    // riding_time on the schedule of ridingTimeHeuristic, riding_time_lp from ridingTimeLp), then
    // write() with that schedule into `path`. Returns false if the file can't be opened.
    bool writeFile(const std::string& path, const std::vector<std::string>& header);

    // Validates internal invariants that a manual pointer/index bug would
    // silently violate: Transfer entries reference the right roles/requests
    // in the right order, every request is either unassigned or fully placed
    // (never both, never partially), and no route exceeds its capacity.
    // Prints every violation found to std::cerr and returns false if any was
    // found -- meant to be called liberally during development, e.g. after
    // every insert/remove, to catch a corruption right where it happens
    // instead of a segfault dozens of iterations later.
    bool checkConsistency() const;

    // The instance's objective (Instance::objective): routeTime() or ridingTimeHeuristic().
    void computeObjectiveFunction();
    // Sum of vehicle driving time on every route, depot-to-first-stop and
    // last-stop-to-depot included (matches the old compute_objective_function).
    int routeTime() const;
    // Users' riding time -- from the pick-up by a vehicle to the drop-off, or to the arrival on foot
    // at the delivery -- summed over the requests served, on the schedule that picks every user up as
    // late as possible and serves everything else as early as possible given that: feasible, cheap
    // (two STP relaxations), not necessarily optimal. Fills `schedule` (route x position) if given.
    // -1 if the routes have no feasible schedule.
    int ridingTimeHeuristic(std::vector<std::vector<int>>* schedule = nullptr) const;
    // The smallest users' riding time over every schedule of these routes: a linear program (CPLEX).
    // Exact with the PT matrix; with the timetable, each tram ride keeps the run of the heuristic
    // schedule (so it is an upper bound, <= ridingTimeHeuristic). -1 if it can't be computed.
    double ridingTimeLp() const;
    int objectiveFunction() const noexcept { return objectiveFunction_; }
    void setObjectiveFunction(int value) noexcept { objectiveFunction_ = value; }

    // Removes every RouteStop belonging to this request from whichever route(s)
    // it's in, drops its Transfer entry if any, and moves it to
    // unassignedRequests(). The primitive every Destroyer is built on top of.
    // Does not re-run propagateForwardDarp -- callers batch-remove first, then
    // propagate once over the affected routes.
    void removeRequest(int requestId);

    // Inserts/removes a stop in one route AND shifts the position fields of
    // every other Transfer referencing that route accordingly. Any code that
    // mutates a route's stop list -- inside Solution or a caller holding a
    // Solution -- must go through these, never Route::insertStop/removeStop
    // directly, or a Transfer elsewhere on that route silently ends up
    // pointing at the wrong stop (see the discussion on why -- this bug was
    // caught by a test with two requests sharing a route).
    void insertStopAt(int routeIndex, int position, RouteStop stop);
    void removeStopAt(int routeIndex, int position);

    int nbUsedRoutes();

private:
    // The STP restricted to a closed set of routes, on integer nodes: 0 is the
    // artificial depot (fixed at the horizon, anchors every absolute window);
    // a route r of the set owns routeOffset[r] (its depot departure, fixed at
    // 0), routeOffset[r] + 1 + i (its stop i) and routeOffset[r] + size + 1
    // (its return to the depot).
    struct DarpGraph {
        // One difference constraint: value(to) - value(from) <= weight.
        struct Arc {
            int from;
            int to;
            int weight;
        };
        // A tram ride with the timetable (TravelTimeProvider::hasTimetable),
        // an arc of variable weight (Molenbruch et al. 2021, EJOR 290, 81-98):
        // some direct run from stop `from` to stop `to` leaving no earlier
        // than value(early) + earlyOffset and arriving no later than
        // value(late) + lateOffset, among those leaving by depCap and arriving
        // from arrFloor on. Runs never overtake each other, so the condition
        // is monotone in both values, and several such arcs sharing the same
        // ride (same stops, caps) hold together iff each one holds.
        struct TramArc {
            int early;
            int earlyOffset;
            int late;
            int lateOffset;
            int from;
            int to;
            int depCap;
            int arrFloor;
        };
        std::vector<int> routeOffset;  // indexed by route, -1 outside the set
        int nbNodes = 1;
        std::vector<Arc> arcs;
        std::vector<TramArc> tramArcs;
        // Start values of the relaxations, from the instance alone: a node's
        // own window when it is a constraint of this STP (the two nodes each
        // request's window and maximum ride time apply to), the depot's
        // return deadline, 0 / the horizon otherwise. Valid lower / upper
        // bounds of every feasible schedule, so starting there gives the same
        // result as starting from 0 / the horizon, in fewer passes.
        std::vector<int> earliestStart;
        std::vector<int> latestStart;
        int stopNode(int route, int position) const { return routeOffset[route] + 1 + position; }
    };

    // Builds every constraint of the STP restricted to affectedRoutes (route
    // chains, depot returns, transfers, windows, maximum ride times), in one
    // pass over their stops and the transfers. Returns false if a request in
    // those routes matches no known pattern.
    bool buildDarpGraph(const std::vector<int>& affectedRoutes, DarpGraph& graph) const;
    // The Bellman-Ford relaxations of the STP, in place from `value`: relaxEarliest pushes values up
    // (earliest schedule above them), relaxLatest pulls them down, `fixed` nodes may not move. False if
    // no schedule exists.
    bool relaxEarliest(const DarpGraph& graph, std::vector<int>& value) const;
    bool relaxLatest(const DarpGraph& graph, std::vector<int>& value, const std::vector<char>& fixed) const;

    // The schedule of ridingTimeHeuristic on the STP of every route, with each user's ride: value(end) +
    // endOffset - value(start), or endTime - value(start) when the end is fixed (walk at delivery with
    // the timetable: the tram run is fixed by the schedule).
    struct RidingSchedule {
        struct Ride {
            int start;
            int end;
            int endOffset = 0;
            bool endFixed = false;
            int endTime = 0;
        };
        DarpGraph graph;
        std::vector<int> value;
        std::vector<Ride> rides;
        int total = 0;
    };
    bool ridingSchedule(RidingSchedule& out) const;

    const Instance* instance_;
    std::vector<Route> routes_;
    std::vector<Transfer> transfers_;
    std::vector<int> unassignedRequests_;
    int objectiveFunction_ = 0;
    // One entry per route, shared by all the routes of a set (and by every
    // copy of this Solution until one of them changes).
    std::vector<std::shared_ptr<const LinkedRoutesTiming>> timing_;
    std::vector<std::shared_ptr<const RouteDarpTiming>> routeTiming_;  // one per route
};