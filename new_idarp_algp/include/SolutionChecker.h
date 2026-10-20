#pragma once

#include <iosfwd>

class Solution;

// Independent feasibility check of a Solution, written directly from the model
// definition: raw Request data (windows, maximum ride time, quantity, loading
// time), drive/walk/PT times, the vehicle capacity and the horizon -- never
// from the derived windows stored in RouteStop, nor from the arcs built by
// Solution::propagateForwardDarp. A bug in the preprocessing or in the STP is
// therefore caught here instead of silently producing an infeasible solution.
//
// Model conventions (all times are start-of-service times, service starting
// only inside the stop's window):
// - every stop of a request, PT stops included, takes the request's loading time;
// - direct / 2-vehicle: ride time = t(delivery) - t(pickup) <= maxRideTime;
// - 2-vehicle: t(stopAfter) >= t(stopBefore) + loadingTime + pt(stopBefore, stopAfter),
//   and pt(stopBefore, stopAfter) >= the minimum time in transit;
// - walk at delivery: the request reaches its delivery point at
//   t(stop) + loadingTime + pt(stop, nearest) + walk, which must lie in the
//   delivery window, and minus t(pickup) be <= maxRideTime;
// - walk at pickup: the request leaves its pickup point at
//   t(stop) - walk - pt(nearest, stop), which must lie in the pickup window
//   (no waiting at the stop), and t(delivery) minus it be <= maxRideTime;
// - every vehicle leaves the depot at 0 and is back by the horizon.
//
// The schedule checked is the earliest one propagateForwardDarp computes
// (ltwDarp), on a copy of `solution`. Unassigned requests are reported but are
// not violations. Prints a summary (plus a few examples per kind of violation)
// to `out` and returns the number of violations found.
int checkSolution(const Solution& solution, std::ostream& out);
