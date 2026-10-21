#pragma once

#include <iosfwd>

class RoutePool;
class Solution;

// Set partitioning over the routes of `pool`, with transfer-compatible
// recombination (CPLEX): selects at most nbVehicles routes, each request
// served once -- entirely by one route (direct, or walk seen as a direct trip
// to/from the PT stop, with that stop's derived window and leg maximum ride
// time), or by one route for each leg of a transfer -- at minimum total
// driving time.
//
// The times are variables (one per stop of every route): within a route
// (chaining with service and driving times, windows, maximum ride time of the
// requests it serves entirely) the constraints always hold, since each route
// alone has a feasible schedule -- the one it had in its solution; between the
// two routes of a transfer (maximum ride time from the pickup to the delivery,
// PT ride from stopBefore to stopAfter), they hold only if both routes are
// selected (big-M, from the bounds of the times). Any stop pair is allowed for
// a transfer, among the candidates of the request or not, as long as its PT
// ride lasts at least the minimum time in transit.
//
// `incumbent` (a complete solution whose routes are all in the pool) is given
// to CPLEX as a starting solution. Returns true if CPLEX found a solution, in
// which case `result` (a fresh Solution(instance)) holds its routes, transfers
// and timing data: its exact schedule is recomputed by Solution::refreshTiming,
// and false is returned if that fails (it never should). A summary is printed
// to `log` if not null; CPLEX's own output is silenced. timeLimit: in
// seconds, none if <= 0 -- on reaching it, the best solution found so far.
bool solveSetPartitioning(const RoutePool& pool, const Solution& incumbent, Solution& result, std::ostream* log,
                          double timeLimit = 0);
