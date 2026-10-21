#pragma once

#include <limits>
#include <vector>

class Solution;

// One route seen as a route of the standard DARP (no transfer), for the exact
// constant-time insertion test of Gschwind & Drexl (2019), "Adaptive Large
// Neighborhood Search with a Constant-Time Feasibility Test for the Dial-a-Ride
// Problem", Transportation Science 53(2), 480-491 (Section 5.2).
//
// Nodes: 0 is the depot departure (fixed at 0), 1 + i the route's stop i,
// size + 1 the return to the depot. Each stop belongs to exactly one pair
// (near, far) of this route bound by a maximum ride time:
// - direct request: (pickup, delivery), the request's maximum ride time;
// - walk pattern: the vehicle leg, the stop's leg maximum ride time;
// - 2-vehicle request, first leg (pickup, stopBefore): maxRideTime -
//   service(stopBefore) - PT time - the duration of the second leg's chain
//   stopAfter -> ... -> delivery on its route as it stands; second leg
//   (stopAfter, delivery): symmetric. ("Dynamic" leg maximum ride times.)
// Windows: [ltwDarp; utwDarp] of each stop, the exact projection of the STP of
// the whole set of linked routes on that stop.
//
// Every one of these constraints is implied by the current STP, and inserting
// a request only adds constraints: a test on this route system never rejects a
// feasible insertion. It is exact for a route linked to no other one; for a
// linked route it is a relaxation (it ignores e.g. minimum time lags between
// stops of this route created by chains of transfers through other routes).
//
// Immutable once built, shared by every copy of a Solution until the route's
// set of linked routes changes (see Solution::refreshTiming).
class RouteDarpTiming {
public:
    // The route's ltwDarp/utwDarp must be up to date.
    RouteDarpTiming(const Solution& solution, int routeIndex);

    int nbNodes() const noexcept { return nbNodes_; }
    int earliest(int node) const { return earliest_[node]; }  // tau_i: as-early-as-possible schedule
    int latest(int node) const { return latest_[node]; }      // tau-bar_i: latest feasible service time
    int serviceTime(int node) const { return serviceTime_[node]; }
    int pointId(int node) const { return pointId_[node]; }
    // Gamma_ab: cumulative travel (service included) along the route from node a to node b (a <= b).
    int cumulativeTravel(int a, int b) const { return cumulative_[b] - cumulative_[a]; }

    // tau-bar_i(t_k): latest feasible service time at node i when node k is
    // served no later than t_k (Proposition 1(ii)); below every feasible time
    // if t_k is earlier than k's earliest time.
    int latestGiven(int i, int k, int tk) const;
    // tau-bar_i(t_k, t_l) (Proposition 1(iii)).
    int latestGiven(int i, int k, int tk, int l, int tl) const {
        int a = latestGiven(i, k, tk), b = latestGiven(i, l, tl);
        return a < b ? a : b;
    }

    static constexpr int kInfeasible = std::numeric_limits<int>::min() / 4;

private:
    int nbNodes_ = 0;
    std::vector<int> earliest_;
    std::vector<int> latest_;
    std::vector<int> serviceTime_;
    std::vector<int> pointId_;
    std::vector<int> cumulative_;
    std::vector<int> latestAtEarliest_;  // nbNodes_ x nbNodes_: [k][i] = tau-bar_i(tau_k)
};
