#include "SetPartitioning.h"

#include <algorithm>
#include <map>
#include <ostream>
#include <vector>

#include <ilcplex/ilocplex.h>

#include "Clock.h"
#include "RoutePool.h"
#include "Solution.h"

namespace {

// One leg of a transfer within a pool route: first leg (pickup, stopBefore)
// or second leg (stopAfter, delivery), by position in the route.
struct LegOccurrence {
    int route;
    int posFirst;
    int posSecond;
};

// Bounds of a route's times, from that route alone: earliest/latest times
// along its chain, given the windows of the stops that carry one. Necessary
// conditions of any feasible schedule, so valid bounds of the variables
// whether the route is selected or not.
struct RouteBounds {
    std::vector<int> lower;
    std::vector<int> upper;
};

// The window the model applies to stop i of a pool route, if any: the two ends
// of a request served entirely by the route (pickup/delivery, or the PT stop
// in place of the walked end, with its derived window -- the same as the
// exact STP), and the pickup and delivery of a transfer, with the request's
// own windows. None on the PT stops of a transfer.
bool windowOf(const Instance& instance, const PoolRoute& route, int i, int& lower, int& upper) {
    const RouteStop& s = route.stops[i];
    if (route.transferLeg[i]) {
        if (s.role == StopRole::StopBefore || s.role == StopRole::StopAfter) return false;
        const TimeWindow& w = (s.role == StopRole::Pickup) ? instance.request(s.requestId).pickupWindow()
                                                           : instance.request(s.requestId).deliveryWindow();
        lower = w.lowerBound;
        upper = w.upperBound;
        return true;
    }
    lower = s.windowLtw;
    upper = s.windowUtw;
    return true;
}

RouteBounds boundsOf(const Instance& instance, const PoolRoute& route) {
    const TravelTimeProvider& tt = instance.travelTimes();
    const int H = instance.horizonTime();
    int n = static_cast<int>(route.stops.size());
    RouteBounds b;
    b.lower.assign(n, 0);
    b.upper.assign(n, H);
    for (int i = 0; i < n; ++i) {
        int lower = 0, upper = H;
        windowOf(instance, route, i, lower, upper);
        b.lower[i] = std::max(0, lower);
        b.upper[i] = std::min(H, upper);
    }
    // Depot departure at 0, return by the horizon (Solution's routes).
    int prevPointId = PointRegistry::depot, prevService = 0, prevTime = 0;
    for (int i = 0; i < n; ++i) {
        const RouteStop& s = route.stops[i];
        b.lower[i] = std::max(b.lower[i], prevTime + prevService + tt.driveTime(prevPointId, s.pointId));
        prevPointId = s.pointId;
        prevService = s.serviceTime;
        prevTime = b.lower[i];
    }
    int nextPointId = PointRegistry::depot, nextTime = H;
    for (int i = n - 1; i >= 0; --i) {
        const RouteStop& s = route.stops[i];
        b.upper[i] = std::min(b.upper[i], nextTime - s.serviceTime - tt.driveTime(s.pointId, nextPointId));
        nextPointId = s.pointId;
        nextTime = b.upper[i];
    }
    return b;
}

// Puts the selected pool routes together into `result`: one vehicle each,
// a Transfer for every request split between two of them -- the windows of
// its PT stops recomputed for the pair actually used, which may come from two
// different solutions -- then the exact schedule.
bool buildSolution(const RoutePool& pool, const std::vector<int>& selected, Solution& result) {
    const Instance& instance = result.instance();
    std::vector<Transfer> transfers(instance.nbRequests());
    std::vector<char> isTransfer(instance.nbRequests(), 0);
    for (int v = 0; v < static_cast<int>(selected.size()); ++v) {
        const PoolRoute& route = pool.route(selected[v]);
        for (int i = 0; i < static_cast<int>(route.stops.size()); ++i) {
            const RouteStop& s = route.stops[i];
            result.insertStopAt(v, i, s);
            if (!route.transferLeg[i]) continue;
            Transfer& t = transfers[s.requestId];
            t.requestId = s.requestId;
            isTransfer[s.requestId] = 1;
            switch (s.role) {
                case StopRole::Pickup: t.routeBefore = v; t.posPickup = i; break;
                case StopRole::StopBefore: t.posStopBefore = i; break;
                case StopRole::StopAfter: t.routeAfter = v; t.posStopAfter = i; break;
                case StopRole::Delivery: t.posDelivery = i; break;
            }
        }
        // Once per request: at its pickup, or at the PT stop of a walk at pickup.
        for (int i = 0; i < static_cast<int>(route.stops.size()); ++i) {
            const RouteStop& s = route.stops[i];
            if (s.role == StopRole::Pickup || (s.role == StopRole::StopAfter && !route.transferLeg[i]))
                result.markAssigned(s.requestId);
        }
    }
    std::vector<int> touched(selected.size());
    for (int v = 0; v < static_cast<int>(selected.size()); ++v) touched[v] = v;
    for (int r = 0; r < instance.nbRequests(); ++r) {
        if (!isTransfer[r]) continue;
        Transfer& t = transfers[r];
        RouteStop& before = result.route(t.routeBefore).stopAt(t.posStopBefore);
        RouteStop& after = result.route(t.routeAfter).stopAt(t.posStopAfter);
        StopPair pair = instance.stopPair(r, before.pointId, after.pointId);
        before.windowLtw = pair.windowBefore.lowerBound;
        before.windowUtw = pair.windowBefore.upperBound;
        after.windowLtw = pair.windowAfter.lowerBound;
        after.windowUtw = pair.windowAfter.upperBound;
        t.ptTravelTime = pair.ptTravelTime;
        result.addTransfer(t);
    }
    bool feasible = result.refreshTiming(touched);
    result.computeObjectiveFunction();
    return feasible && result.unassignedRequests().empty();
}

}  // namespace

bool solveSetPartitioning(const RoutePool& pool, const Solution& incumbent, Solution& result, std::ostream* log,
                          double timeLimit) {
    const Instance& instance = incumbent.instance();
    const TravelTimeProvider& tt = instance.travelTimes();
    const int nbRoutes = pool.size();
    TimePoint start = now();

    // Who serves what: requests entirely in a route (a_rj), first legs
    // (a1_rj) and second legs (a2_rj) of transfers.
    std::vector<std::vector<int>> entire(instance.nbRequests());
    std::vector<std::vector<LegOccurrence>> firstLegs(instance.nbRequests()), secondLegs(instance.nbRequests());
    std::vector<RouteBounds> bounds(nbRoutes);
    for (int j = 0; j < nbRoutes; ++j) {
        const PoolRoute& route = pool.route(j);
        bounds[j] = boundsOf(instance, route);
        int n = static_cast<int>(route.stops.size());
        for (int i = 0; i < n; ++i) {
            const RouteStop& s = route.stops[i];
            bool opensLeg = s.role == StopRole::Pickup || s.role == StopRole::StopAfter;
            if (!opensLeg) continue;
            int k = i + 1;
            while (route.stops[k].requestId != s.requestId) ++k;
            if (!route.transferLeg[i]) entire[s.requestId].push_back(j);
            else if (s.role == StopRole::Pickup) firstLegs[s.requestId].push_back({j, i, k});
            else secondLegs[s.requestId].push_back({j, i, k});
        }
    }

    IloEnv env;
    bool found = false;
    long long nbLinks = 0, nbStopPairs = 0;
    try {
        IloModel model(env);
        IloBoolVarArray x(env, nbRoutes);
        std::vector<IloNumVarArray> t;
        t.reserve(nbRoutes);
        IloExpr objective(env);
        IloExpr nbSelected(env);
        for (int j = 0; j < nbRoutes; ++j) {
            const PoolRoute& route = pool.route(j);
            int n = static_cast<int>(route.stops.size());
            objective += route.cost * x[j];
            nbSelected += x[j];
            t.emplace_back(env, n);
            for (int i = 0; i < n; ++i) t[j][i] = IloNumVar(env, bounds[j].lower[i], bounds[j].upper[i]);

            // Within the route (10)-(11), always: chaining (depot departure at
            // 0 and return by the horizon are in the bounds), windows (bounds)
            // and maximum ride time of the requests it serves entirely.
            for (int i = 0; i + 1 < n; ++i) {
                const RouteStop& s = route.stops[i];
                model.add(t[j][i + 1] - t[j][i] >= s.serviceTime + tt.driveTime(s.pointId, route.stops[i + 1].pointId));
            }
            for (int i = 0; i < n; ++i) {
                const RouteStop& s = route.stops[i];
                if (route.transferLeg[i] || !(s.role == StopRole::Pickup || s.role == StopRole::StopAfter)) continue;
                int k = i + 1;
                while (route.stops[k].requestId != s.requestId) ++k;
                const Request& request = instance.request(s.requestId);
                bool walk = s.role == StopRole::StopAfter || route.stops[k].role == StopRole::StopBefore;
                int maxRideTime = !walk ? request.maxRideTime()
                                  : (s.role == StopRole::StopAfter)
                                      ? request.walkAccess().maxRideTimeForStop(s.pointId)
                                      : request.walkAccess().maxRideTimeForStop(route.stops[k].pointId);
                model.add(t[j][k] - t[j][i] <= maxRideTime);
            }
        }
        model.add(IloMinimize(env, objective));
        model.add(nbSelected <= instance.nbVehicles());  // (12)
        objective.end();
        nbSelected.end();

        for (int r = 0; r < instance.nbRequests(); ++r) {
            // (6): served once, entirely or through its first leg; (7): as
            // many second legs as first legs.
            IloExpr served(env), balance(env);
            for (int j : entire[r]) served += x[j];
            for (const LegOccurrence& leg : firstLegs[r]) {
                served += x[leg.route];
                balance += x[leg.route];
            }
            for (const LegOccurrence& leg : secondLegs[r]) balance -= x[leg.route];
            model.add(served == 1);
            if (!firstLegs[r].empty() || !secondLegs[r].empty()) model.add(balance == 0);
            served.end();
            balance.end();

            // (8)-(9), in linear size rather than for every (first leg,
            // second leg) pair of routes: TP, TSB, TSA, TD stand for the times
            // of the pickup, stopBefore, stopAfter and delivery in the legs
            // selected, and are only bounded on the side that matters --
            // TP <= t(pickup), TSB >= t(stopBefore) for the first leg selected,
            // TSA <= t(stopAfter), TD >= t(delivery) for the second one. Then
            // TD - TP <= maxRideTime and, per stop pair (a, b),
            // TSA - TSB >= pt(a, b) + service time once a first leg ending
            // at a and a second leg starting at b are selected: exactly (8)
            // and (9) for the two legs selected. Big-M from the time bounds.
            // Linear size: |first legs| + |second legs| + |stop pairs|
            // constraints, where the pairs of routes ran into the tens of
            // millions (15,000 routes after 30 s on a 30-request instance).
            if (firstLegs[r].empty() || secondLegs[r].empty()) continue;
            const Request& request = instance.request(r);
            // Bounds of TP, TSB, TSA, TD -- the big-M follow from them. TP and
            // TD within the request's pickup and delivery windows (those of its
            // pickup and delivery in any leg); TSB and TSA within the hull of
            // the bounds of the stopBefore / stopAfter of its legs. They must
            // leave TD - TP <= maxRideTime satisfiable when no leg is selected
            // (request served by a single route): ltwD - utwP <= maxRideTime
            // holds for any request served at all, but TP's upper bound is
            // raised if needed.
            const TimeWindow& pickupWindow = request.pickupWindow();
            const TimeWindow& deliveryWindow = request.deliveryWindow();
            int lbTP = pickupWindow.lowerBound;
            int ubTP = std::max(pickupWindow.upperBound, deliveryWindow.lowerBound - request.maxRideTime());
            int lbTD = deliveryWindow.lowerBound, ubTD = deliveryWindow.upperBound;
            int lbTSB = instance.horizonTime(), ubTSB = 0, lbTSA = instance.horizonTime(), ubTSA = 0;
            for (const LegOccurrence& leg : firstLegs[r]) {
                lbTSB = std::min(lbTSB, bounds[leg.route].lower[leg.posSecond]);
                ubTSB = std::max(ubTSB, bounds[leg.route].upper[leg.posSecond]);
            }
            for (const LegOccurrence& leg : secondLegs[r]) {
                lbTSA = std::min(lbTSA, bounds[leg.route].lower[leg.posFirst]);
                ubTSA = std::max(ubTSA, bounds[leg.route].upper[leg.posFirst]);
            }
            IloNumVar TP(env, lbTP, ubTP), TSB(env, lbTSB, ubTSB), TSA(env, lbTSA, ubTSA), TD(env, lbTD, ubTD);
            model.add(TD - TP <= request.maxRideTime());  // (8)

            // Selected legs by PT stop: endingAt[a] = sum of x over the first
            // legs ending at a, startingAt[b] over the second legs starting at b.
            std::map<int, IloExpr> endingAt, startingAt;
            for (const LegOccurrence& leg : firstLegs[r]) {
                const RouteBounds& b = bounds[leg.route];
                IloBoolVar selected = x[leg.route];
                model.add(TP <= t[leg.route][leg.posFirst] + (ubTP - b.lower[leg.posFirst]) * (1 - selected));
                model.add(TSB >= t[leg.route][leg.posSecond] - (b.upper[leg.posSecond] - lbTSB) * (1 - selected));
                nbLinks += 2;
                int stop = pool.route(leg.route).stops[leg.posSecond].pointId;
                auto it = endingAt.find(stop);
                if (it == endingAt.end()) it = endingAt.emplace(stop, IloExpr(env)).first;
                it->second += selected;
            }
            for (const LegOccurrence& leg : secondLegs[r]) {
                const RouteBounds& b = bounds[leg.route];
                IloBoolVar selected = x[leg.route];
                model.add(TSA <= t[leg.route][leg.posFirst] + (ubTSA - b.lower[leg.posFirst]) * (1 - selected));
                model.add(TD >= t[leg.route][leg.posSecond] - (b.upper[leg.posSecond] - lbTD) * (1 - selected));
                nbLinks += 2;
                int stop = pool.route(leg.route).stops[leg.posFirst].pointId;
                auto it = startingAt.find(stop);
                if (it == startingAt.end()) it = startingAt.emplace(stop, IloExpr(env)).first;
                it->second += selected;
            }
            for (auto& [a, ending] : endingAt) {
                for (auto& [b, starting] : startingAt) {
                    int pt = tt.publicTransportTime(a, b);
                    if (pt < tt.minimumTimeInTransit()) {
                        model.add(ending + starting <= 1);
                        ++nbStopPairs;
                        continue;
                    }
                    int transferTime = pt + request.loadingTime();
                    // TSA - TSB >= lbTSA - ubTSB always: nothing to add if
                    // that is enough, else the big-M that relaxes down to it.
                    int bigM = transferTime - (lbTSA - ubTSB);
                    if (bigM <= 0) continue;
                    model.add(TSA - TSB >= transferTime - bigM * (2 - ending - starting));  // (9)
                    ++nbStopPairs;
                }
            }
            for (auto& entry : endingAt) entry.second.end();
            for (auto& entry : startingAt) entry.second.end();
        }

        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());
        if (timeLimit > 0) cplex.setParam(IloCplex::Param::TimeLimit, timeLimit);

        // Starting solution: the incumbent's routes, with its earliest schedule.
        IloNumVarArray startVars(env);
        IloNumArray startValues(env);
        std::vector<char> inStart(nbRoutes, 0);
        for (int v = 0; v < incumbent.nbRoutes(); ++v) {
            if (incumbent.route(v).empty()) continue;
            int j = pool.find(incumbent, v);
            if (j < 0) continue;
            inStart[j] = 1;
            for (int i = 0; i < incumbent.route(v).size(); ++i) {
                startVars.add(t[j][i]);
                startValues.add(incumbent.route(v).stopAt(i).ltwDarp);
            }
        }
        for (int j = 0; j < nbRoutes; ++j) {
            startVars.add(x[j]);
            startValues.add(inStart[j]);
        }
        cplex.addMIPStart(startVars, startValues, IloCplex::MIPStartAuto);
        startVars.end();
        startValues.end();

        if (cplex.solve()) {
            std::vector<int> selected;
            for (int j = 0; j < nbRoutes; ++j)
                if (cplex.getValue(x[j]) > 0.5) selected.push_back(j);
            found = buildSolution(pool, selected, result);
            if (log) {
                *log << "[setPartitioning] " << nbRoutes << " routes, " << nbLinks << " leg links, " << nbStopPairs
                     << " stop pair constraints -- " << cplex.getStatus() << ", objective "
                     << cplex.getObjValue() << " (bound " << cplex.getBestObjValue() << "), " << selected.size()
                     << " routes, " << secondsSince(start) << " s";
                if (!found) *log << " -- INFEASIBLE once put together";
                *log << "\n";
            }
        } else if (log) {
            *log << "[setPartitioning] " << nbRoutes << " routes -- no solution (" << cplex.getStatus() << ")\n";
        }
    } catch (IloException& e) {
        if (log) *log << "[setPartitioning] CPLEX exception: " << e << "\n";
        found = false;
    }
    env.end();
    return found;
}
