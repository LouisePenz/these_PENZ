#include "RouteDarpTiming.h"

#include <algorithm>
#include <iostream>

#include "Solution.h"

namespace {
// Duration of the chain from stop i to stop j (i <= j) of `route`: services and
// drives in between, no waiting -- a lower bound on t(j) - t(i).
int chainDuration(const Route& route, const TravelTimeProvider& tt, int i, int j) {
    int duration = 0;
    for (int x = i; x < j; ++x)
        duration += route.stopAt(x).serviceTime + tt.driveTime(route.stopAt(x).pointId, route.stopAt(x + 1).pointId);
    return duration;
}
}  // namespace

RouteDarpTiming::Inputs RouteDarpTiming::inputsOf(const Solution& solution, int routeIndex) {
    const Route& route = solution.route(routeIndex);
    const Instance& instance = solution.instance();
    const TravelTimeProvider& tt = instance.travelTimes();
    const int n = route.size();
    const int nbNodes = n + 2;

    // Node data and windows.
    Inputs in;
    in.pointId.assign(nbNodes, route.depotPointId());
    in.serviceTime.assign(nbNodes, 0);
    in.lower.assign(nbNodes, 0);
    in.upper.assign(nbNodes, 0);
    in.upper[n + 1] = route.depotReturnDeadline();  // depot departure: [0; 0]
    for (int i = 0; i < n; ++i) {
        const RouteStop& s = route.stopAt(i);
        in.pointId[i + 1] = s.pointId;
        in.serviceTime[i + 1] = s.serviceTime;
        in.lower[i + 1] = s.ltwDarp;
        in.upper[i + 1] = s.utwDarp;
    }

    // One maximum-ride-time pair per request leg on this route (see the header).
    auto findOnRoute = [&route](int requestId, StopRole role) {
        for (int i = 0; i < route.size(); ++i)
            if (route.stopAt(i).requestId == requestId && route.stopAt(i).role == role) return i;
        return -1;
    };
    for (int i = 0; i < n; ++i) {
        const RouteStop& s = route.stopAt(i);
        const Request& request = instance.request(s.requestId);
        auto transfer = std::find_if(solution.transfers().begin(), solution.transfers().end(),
                                     [&s](const Transfer& t) { return t.requestId == s.requestId; });
        if (transfer != solution.transfers().end()) {
            const Transfer& t = *transfer;
            int stopBeforeService = solution.route(t.routeBefore).stopAt(t.posStopBefore).serviceTime;
            if (s.role == StopRole::Pickup) {
                int secondLeg = chainDuration(solution.route(t.routeAfter), tt, t.posStopAfter, t.posDelivery);
                in.lags.push_back({t.posPickup + 1, t.posStopBefore + 1,
                                   request.maxRideTime() - stopBeforeService - t.ptTravelTime - secondLeg});
            } else if (s.role == StopRole::StopAfter) {
                int firstLeg = chainDuration(solution.route(t.routeBefore), tt, t.posPickup, t.posStopBefore);
                in.lags.push_back({t.posStopAfter + 1, t.posDelivery + 1,
                                   request.maxRideTime() - firstLeg - stopBeforeService - t.ptTravelTime});
            }
        } else if (s.role == StopRole::Pickup) {
            int delivery = findOnRoute(s.requestId, StopRole::Delivery);
            if (delivery >= 0) {
                in.lags.push_back({i + 1, delivery + 1, request.maxRideTime()});
            } else {
                int stop = findOnRoute(s.requestId, StopRole::StopBefore);  // walk at delivery
                in.lags.push_back(
                    {i + 1, stop + 1, request.walkAccess().maxRideTimeForStop(route.stopAt(stop).pointId)});
            }
        } else if (s.role == StopRole::StopAfter) {  // walk at pickup
            int delivery = findOnRoute(s.requestId, StopRole::Delivery);
            in.lags.push_back({i + 1, delivery + 1, request.walkAccess().maxRideTimeForStop(s.pointId)});
        }
    }
    return in;
}

RouteDarpTiming::RouteDarpTiming(const TravelTimeProvider& tt, Inputs inputs) : inputs_(std::move(inputs)) {
    nbNodes_ = static_cast<int>(inputs_.pointId.size());
    const std::vector<int>& lower = inputs_.lower;
    const std::vector<int>& upper = inputs_.upper;
    const std::vector<MaxLag>& lags = inputs_.lags;

    std::vector<int> travel(nbNodes_ - 1);  // node m -> m + 1, service of m included
    cumulative_.assign(nbNodes_, 0);
    for (int m = 0; m + 1 < nbNodes_; ++m) {
        travel[m] = inputs_.serviceTime[m] + tt.driveTime(inputs_.pointId[m], inputs_.pointId[m + 1]);
        cumulative_[m + 1] = cumulative_[m] + travel[m];
    }

    // Relaxations on this route alone (small): earliest from the lower
    // bounds, latest from the upper bounds -- optionally with node `cap`
    // served no later than capValue.
    auto earliestSchedule = [&](std::vector<int>& v) {
        v = lower;
        for (int pass = 0; pass <= nbNodes_; ++pass) {
            bool changed = false;
            for (int m = 0; m + 1 < nbNodes_; ++m) {
                if (v[m + 1] < v[m] + travel[m]) {
                    v[m + 1] = v[m] + travel[m];
                    changed = true;
                }
            }
            for (const MaxLag& g : lags) {
                if (v[g.near] < v[g.far] - g.value) {
                    v[g.near] = v[g.far] - g.value;
                    changed = true;
                }
            }
            if (!changed) break;
        }
    };
    auto latestSchedule = [&](std::vector<int>& v, int cap, int capValue) {
        v = upper;
        if (cap >= 0) v[cap] = std::min(v[cap], capValue);
        for (int pass = 0; pass <= nbNodes_; ++pass) {
            bool changed = false;
            for (int m = nbNodes_ - 2; m >= 0; --m) {
                if (v[m] > v[m + 1] - travel[m]) {
                    v[m] = v[m + 1] - travel[m];
                    changed = true;
                }
            }
            for (const MaxLag& g : lags) {
                if (v[g.far] > v[g.near] + g.value) {
                    v[g.far] = v[g.near] + g.value;
                    changed = true;
                }
            }
            if (!changed) break;
        }
    };

    earliestSchedule(earliest_);
    latestSchedule(latest_, -1, 0);
    for (int m = 0; m < nbNodes_; ++m) {
        if (earliest_[m] > latest_[m]) {
            std::cerr << "[RouteDarpTiming] route infeasible as a DARP route -- windows out of date?\n";
            break;
        }
    }

    // tau-bar_i(tau_k) for every pair: latest schedule with k capped at its earliest time.
    latestAtEarliest_.assign(static_cast<size_t>(nbNodes_) * nbNodes_, 0);
    std::vector<int> v;
    for (int k = 0; k < nbNodes_; ++k) {
        latestSchedule(v, k, earliest_[k]);
        std::copy(v.begin(), v.end(), latestAtEarliest_.begin() + static_cast<size_t>(k) * nbNodes_);
    }
}

int RouteDarpTiming::latestGiven(int i, int k, int tk) const {
    if (tk < earliest_[k]) return kInfeasible;
    int bound = latestAtEarliest_[static_cast<size_t>(k) * nbNodes_ + i] + (tk - earliest_[k]);
    return std::min(bound, latest_[i]);
}
