#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Route.h"

class Solution;

// One route of the pool: its stops as in the solution it was taken from
// (ltwDarp/utwDarp are that solution's and mean nothing here -- the set
// partitioning gives the route new times), plus, per stop, whether it belongs
// to a 2-vehicle request: Pickup + StopBefore is either the first leg of a
// transfer or a trip walked at delivery, and only the solution's Transfer
// records tell them apart.
struct PoolRoute {
    std::vector<RouteStop> stops;
    std::vector<char> transferLeg;
    int cost = 0;  // driving time, depot to depot (as Solution::computeObjectiveFunction)
};

// The routes collected from the solutions met during the search, each kept
// once: a route is identified by its sequence of (role, request, point,
// transfer leg or not) -- its windows, ride times and cost all follow from it.
class RoutePool {
public:
    // Adds every non-empty route of `solution` not in the pool yet.
    void add(const Solution& solution);

    void clear() {
        routes_.clear();
        index_.clear();
    }

    int size() const noexcept { return static_cast<int>(routes_.size()); }
    const PoolRoute& route(int index) const { return routes_[index]; }
    // Index of the route `routeIndex` of `solution` in the pool, -1 if absent.
    int find(const Solution& solution, int routeIndex) const;

private:
    using Key = std::vector<std::uint64_t>;
    struct KeyHash {
        std::size_t operator()(const Key& key) const noexcept;
    };
    static Key keyOf(const Solution& solution, int routeIndex, std::vector<char>& transferLeg);

    std::vector<PoolRoute> routes_;
    std::unordered_map<Key, int, KeyHash> index_;
};
