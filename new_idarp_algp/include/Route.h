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
// transfer, not just this one -- the earliest/latest times of the exact STP,
// written by Solution::propagateForwardDarp/propagateBackwardDarp, never by
// Route itself.
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

    int ltwDarp = 0;
    int utwDarp = 0;
};

// A single vehicle's route: an ordered list of RouteStop, stored contiguously
// (not a linked list -- see the discussion on why: routes are short, ~20
// stops, so the O(n) shift cost of vector insert/erase is dominated by the
// O(n) window recomputation every insertion triggers anyway, and contiguous
// storage avoids the whole class of dangling-pointer bugs the old code had).
//
// NOTE: the exact, transfer-aware DARP feasibility test lives in
// Solution::propagateForwardDarp, and the O(1) pre-filter in
// InsertionEvaluator (both need to see every route linked by a transfer, not
// just this one).
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

private:
    int vehicleId_;
    int capacity_;
    int depotPointId_;
    int depotDeparture_;
    int depotReturnDeadline_;

    std::vector<RouteStop> stops_;
    std::vector<int> loadAfterStop_;  // same size as stops_, kept in sync by recomputeLoad()
};
