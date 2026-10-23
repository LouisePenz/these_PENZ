#pragma once

#include <limits>
#include <vector>

class Solution;

// Preprocessed timing data of one set of linked routes -- routes connected by
// a Transfer, directly or through other routes of the set -- for the
// constant-time insertion test of Masson, Lehuédé & Péton (2013), "Efficient
// feasibility testing for request insertion in the pickup and delivery
// problem with transfers", Operations Research Letters 41, 211-215.
//
// Nodes are the paper's precedence graph G_s: every stop of these routes, plus
// each route's start and end depot. Arcs are the route successions and, for
// every Transfer, stopBefore -> stopAfter (service at stopBefore + PT time).
//
// Adaptations to this DARP:
// - h (earliest) and l (latest) are the exact STP times ltwDarp / utwDarp,
//   which also account for the maximum ride times; the paper uses the
//   as-early-as-possible schedule and the static windows. Since maximum ride
//   times can push a stop later than any precedence path predicts, the test
//   built on this data is a NECESSARY condition only -- the exact STP check
//   (Solution::propagateForwardDarp) remains the final word.
// - the slack of an arc is the waiting it absorbs (h_to - h_from - s_from -
//   travel), not the waiting at its terminal vertex: a stopAfter has two
//   predecessors, and the paper's per-vertex waiting would understate the
//   slack of the non-binding one.
//
// Immutable once built: every copy of a Solution shares it until one of its
// routes changes (see Solution::refreshTiming).
class LinkedRoutesTiming {
public:
    static constexpr int kUnreachable = std::numeric_limits<int>::max() / 4;

    // `routes` must be one whole set of linked routes (Solution::dependentRoutes)
    // whose ltwDarp/utwDarp are up to date.
    LinkedRoutesTiming(const Solution& solution, const std::vector<int>& routes);

    // Node of `position` in `route`: -1 is the start depot, route size the end depot.
    int node(int route, int position) const { return routeOffset_[route] + position + 1; }

    int earliest(int node) const { return earliest_[node]; }
    int latest(int node) const { return latest_[node]; }
    int serviceTime(int node) const { return serviceTime_[node]; }
    int pointId(int node) const { return pointId_[node]; }

    // F_u (Proposition 2): how far service at `node` can be postponed without
    // pushing it or any of its successors past its latest time.
    int forwardSlack(int node) const { return forwardSlack_[node]; }

    // ST_{from,to} (Definition 2): minimum total slack over the paths from ->
    // to, kUnreachable if there is none. ST_{u,u} = 0.
    int slackTime(int from, int to) const { return slackTime_[static_cast<size_t>(from) * nbNodes_ + to]; }
    bool reaches(int from, int to) const { return slackTime(from, to) != kUnreachable; }

private:
    int nbNodes_ = 0;
    std::vector<int> routeOffset_;  // indexed by route index of the whole solution; -1 outside this set
    std::vector<int> earliest_;
    std::vector<int> latest_;
    std::vector<int> serviceTime_;
    std::vector<int> pointId_;
    std::vector<int> forwardSlack_;
    std::vector<int> slackTime_;  // nbNodes_ x nbNodes_, row = from
};
