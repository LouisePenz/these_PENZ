#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "Instance.h"
#include "Route.h"
#include "Transfer.h"

class LinkedRoutesTiming;

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

    // A node in the constraint graph: (routeIndex, position), where position
    // -1 means "that route's own depot" (departure), kRouteEndPosition (-2)
    // its return to the depot, and (-1, -1) is the single shared artificial
    // depot used to anchor every request's absolute time window.
    using NodeRef = std::pair<int, int>;

    // The two nodes and the maximum-ride-time value a request's direct/2-vehicle/
    // walk pattern reduces to (see the discussion on why every pattern collapses
    // into this single shape once walk/PT durations are folded into derived
    // windows).
    struct RideTimeLink {
        NodeRef nearNode;
        NodeRef farNode;
        int maxRideTime;
    };

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
    // `overrides` lets a caller supply the RideTimeLink for specific requests
    // directly instead of letting it be auto-detected from the roles present
    // in affectedRoutes -- needed when testing one leg of a 2-vehicle request
    // in isolation (Pickup+StopBefore only, no partner yet), which would
    // otherwise be silently mistaken for a walk pattern. See the discussion
    // on why InsertionEvaluator needs this rather than a hidden flag on
    // RouteStop.
    //
    // On success, updates RouteStop::ltwDarp (earliest feasible time) for
    // every stop in `affectedRoutes` and returns true. On failure, returns
    // false; the ltwDarp values may be left inconsistent and must not be
    // trusted -- re-run or discard the change that triggered this call.
    bool propagateForwardDarp(const std::vector<int>& affectedRoutes,
                               const std::vector<std::pair<int, RideTimeLink>>& overrides = {});

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

    // To call after any change to the routes or transfers: on every set of
    // linked routes containing one of `changedRoutes` -- as they are NOW, so a
    // removed transfer that split a set, or an added one that merged two, is
    // handled -- recomputes the loads, re-runs the exact STP forward and
    // backward (ltwDarp/utwDarp), and rebuilds the timing data. Returns false
    // if one of those sets has no feasible schedule.
    bool refreshTiming(const std::vector<int>& changedRoutes);

    // Human-readable dump of the current state (routes, stops, transfers,
    // unassigned requests) -- deliberately not the request/instance data
    // already printed by Instance::print().
    void print() const;

    // Validates internal invariants that a manual pointer/index bug would
    // silently violate: Transfer entries reference the right roles/requests
    // in the right order, every request is either unassigned or fully placed
    // (never both, never partially), and no route exceeds its capacity.
    // Prints every violation found to std::cerr and returns false if any was
    // found -- meant to be called liberally during development, e.g. after
    // every insert/remove, to catch a corruption right where it happens
    // instead of a segfault dozens of iterations later.
    bool checkConsistency() const;

    // Sum of vehicle driving time on every route, depot-to-first-stop and
    // last-stop-to-depot included (matches the old compute_objective_function).
    void computeObjectiveFunction();
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
    static const NodeRef kArtificialDepot;
    static const int kRouteEndPosition;

    struct NodeRefHash {
        size_t operator()(const NodeRef& n) const noexcept {
            // n.first can be -1 (artificial depot, or a route's own depot
            // position uses -1 for n.second too) -- shifting a negative
            // signed value is undefined behavior, so reinterpret as unsigned
            // first (bit pattern preserved, just no longer "negative").
            long long high = static_cast<long long>(static_cast<unsigned int>(n.first));
            return std::hash<long long>()((high << 32) ^ static_cast<unsigned int>(n.second));
        }
    };

    // One difference constraint: value(to) - value(from) <= weight.
    struct Arc {
        NodeRef from;
        NodeRef to;
        int weight;
    };

    // Builds every constraint of the STP restricted to affectedRoutes (route
    // chains, depot returns, transfers, windows, maximum ride times). Returns
    // false if a request in those routes matches no known pattern.
    bool buildDarpArcs(const std::vector<int>& affectedRoutes,
                       const std::vector<std::pair<int, RideTimeLink>>& overrides, std::vector<Arc>& arcs) const;

    // Auto-detects the RideTimeLink for a request from the roles present in
    // affectedRoutes. Returns false if the request isn't (fully) present.
    bool findRideTimeLink(int requestId, const std::vector<int>& affectedRoutes, RideTimeLink& outLink) const;

    const RouteStop& stopAtNode(NodeRef node) const { return routes_[node.first].stopAt(node.second); }

    const Instance* instance_;
    std::vector<Route> routes_;
    std::vector<Transfer> transfers_;
    std::vector<int> unassignedRequests_;
    int objectiveFunction_ = 0;
    // One entry per route, shared by all the routes of a set (and by every
    // copy of this Solution until one of them changes).
    std::vector<std::shared_ptr<const LinkedRoutesTiming>> timing_;
};