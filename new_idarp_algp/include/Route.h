#pragma once

#include <vector>

#include "TravelTimeProvider.h"

// Role of a stop within a route. The depot itself is never stored in the
// route's stop list -- it's an implicit boundary (see Route::pointBefore/After).
enum class StopRole { Pickup, Delivery, StopBefore, StopAfter };

// One event in a route: board or alight at `pointId`, for leg `legIndex` of
// request `requestId`. windowLtw/windowUtw are the *static* time window at
// this specific point (from the request/AccessOption, already accounting for
// the maximum ride time where relevant -- see Instance's precomputation).
//
// ltwDarp/utwDarp are *dynamic* and depend on every route linked by a
// transfer, not just this one -- written by Solution::propagateForwardDarp,
// never by Route itself.
//
// ltwPdp/utwPdp/forwardTimeSlackPdp are also dynamic, but *local*: written by
// Route::recomputePdpWindows/recomputeForwardTimeSlack, valid only when this
// route has no active Transfer (see the discussion on why -- it ignores the
// maximum-ride-time constraint and any cross-route arc on purpose, as a fast
// pre-filter, not the exact check).
struct RouteStop {
    StopRole role = StopRole::Pickup;
    int requestId = -1;
    int legIndex = 0;
    int pointId = -1;
    int serviceTime = 0;
    // Signed capacity change at this stop: +quantity when boarding (Pickup,
    // StopAfter), -quantity when alighting (Delivery, StopBefore).
    int loadDelta = 0;

    int windowLtw = 0;
    int windowUtw = 0;

    int ltwPdp = 0;
    int utwPdp = 0;
    int ltwDarp = 0;
    int utwDarp = 0;
    int forwardTimeSlackPdp = 0;
};

// A single vehicle's route: an ordered list of RouteStop, stored contiguously
// (not a linked list -- see the discussion on why: routes are short, ~20
// stops, so the O(n) shift cost of vector insert/erase is dominated by the
// O(n) window recomputation every insertion triggers anyway, and contiguous
// storage avoids the whole class of dangling-pointer bugs the old code had).
//
// NOTE: the exact, transfer-aware DARP feasibility test lives in
// Solution::propagateForwardDarp (it needs to see every route linked by a
// transfer, not just this one). Route only offers the local, single-route PDP
// pre-filter below.
class Route {
public:
    Route(int vehicleId, int capacity, int depotPointId, int depotDeparture, int depotReturnDeadline);

    int vehicleId() const noexcept { return vehicleId_; }
    int capacity() const noexcept { return capacity_; }
    int depotPointId() const noexcept { return depotPointId_; }
    int depotDeparture() const noexcept { return depotDeparture_; }
    int depotReturnDeadline() const noexcept { return depotReturnDeadline_; }

    int size() const noexcept { return static_cast<int>(stops_.size()); }
    bool empty() const noexcept { return stops_.empty(); }

    const RouteStop& stopAt(int position) const { return stops_[position]; }
    RouteStop& stopAt(int position) { return stops_[position]; }
    const std::vector<RouteStop>& stops() const noexcept { return stops_; }

    // Point id of the vertex right before/after a given insertion position
    // (0 == right after the depot). Returns the depot's point id at the
    // boundaries.
    int pointBefore(int position) const;
    int pointAfter(int position) const;

    // Inserts a stop so that it ends up at index `position` (shifts the rest
    // of the route right). Does not update any time window or load data --
    // callers must call recomputeLoad() and the window propagation afterwards.
    void insertStop(int position, RouteStop stop);
    // Removes the stop currently at `position`.
    void removeStop(int position);

    // Cumulative load after each position, local to this route -- replaces
    // the old solution-wide free_load matrix (which was indexed by
    // position_in_route across all routes at once, an easy source of
    // confusion between routes).
    void recomputeLoad();
    int loadAfter(int position) const;
    // True if the vehicle has enough free capacity to carry `extraLoad`
    // additional units from just after posFrom to just after posTo (inclusive).
    bool hasFreeCapacity(int posFrom, int posTo, int extraLoad) const;

    // PDP (Masson, Lehuédé, Péton) preprocessing for THIS route in isolation
    // -- ignores the maximum-ride-time constraint and any transfer to another
    // route on purpose (valid, for now, only when this route has no active
    // Transfer -- e.g. a direct-request insertion). Fills ltwPdp/utwPdp
    // (forward/backward pass), then forwardTimeSlackPdp (Savelsbergh 1992's
    // recursive formula). Call recomputePdpWindows() first.
    void recomputePdpWindows(const TravelTimeProvider& travelTimes);
    void recomputeForwardTimeSlack(const TravelTimeProvider& travelTimes);

    // O(1) necessary-condition check, using the precomputed PDP data above:
    // would inserting a stop (pointId, serviceTime, window) at insertion
    // position `position` violate any time window on this route? true is not
    // a guarantee of feasibility (ride-time-max and capacity still need the
    // exact check); false means "definitely infeasible on time windows alone
    // -- skip the expensive check". outArrival/outDelta, if given, receive
    // the stop's own PDP-estimated service time and the delay it imposes on
    // the route's existing successor (0 if none) -- used as cheap proxies
    // for building a Pareto front before running the exact check (see the
    // discussion on why these are optimistic estimates, not exact values).
    bool pdpAdmitsInsertion(const TravelTimeProvider& travelTimes, int position, int pointId, int serviceTime,
                             int windowLtw, int windowUtw, int* outArrival = nullptr, int* outDelta = nullptr) const;

    // Same idea, for inserting TWO stops back to back at the same gap
    // (position): first, then second, with no existing route stop between
    // them. Needed because pdpAdmitsInsertion alone can't validate this case
    // -- the two new stops share the same predecessor/successor pair from
    // the original route, not independent gaps. outSecondArrival/outDelta as
    // above, but for the SECOND stop (e.g. the one that plays "dropoff" in a
    // Pareto front).
    bool pdpAdmitsAdjacentInsertion(const TravelTimeProvider& travelTimes, int position, int firstPointId,
                                     int firstServiceTime, int firstWindowLtw, int firstWindowUtw, int secondPointId,
                                     int secondServiceTime, int secondWindowLtw, int secondWindowUtw,
                                     int* outSecondArrival = nullptr, int* outDelta = nullptr) const;

private:
    int vehicleId_;
    int capacity_;
    int depotPointId_;
    int depotDeparture_;
    int depotReturnDeadline_;

    std::vector<RouteStop> stops_;
    std::vector<int> loadAfterStop_;  // same size as stops_, kept in sync by recomputeLoad()
};
