#include "LinkedRoutesTiming.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "Solution.h"

LinkedRoutesTiming::LinkedRoutesTiming(const Solution& solution, const std::vector<int>& routes) {
    const TravelTimeProvider& tt = solution.instance().travelTimes();

    routeOffset_.assign(solution.nbRoutes(), -1);
    for (int r : routes) {
        routeOffset_[r] = nbNodes_;
        nbNodes_ += solution.route(r).size() + 2;
    }
    earliest_.assign(nbNodes_, 0);
    latest_.assign(nbNodes_, 0);
    serviceTime_.assign(nbNodes_, 0);
    pointId_.assign(nbNodes_, 0);

    // Node data. The start depot is fixed (departure at 0, never postponed);
    // the end depot's earliest time is the return from the last stop.
    for (int r : routes) {
        const Route& route = solution.route(r);
        int start = node(r, -1);
        pointId_[start] = route.depotPointId();
        for (int i = 0; i < route.size(); ++i) {
            const RouteStop& s = route.stopAt(i);
            int u = node(r, i);
            earliest_[u] = s.ltwDarp;
            latest_[u] = s.utwDarp;
            serviceTime_[u] = s.serviceTime;
            pointId_[u] = s.pointId;
        }
        int end = node(r, route.size());
        int last = end - 1;
        pointId_[end] = route.depotPointId();
        earliest_[end] = route.empty() ? 0
                                       : earliest_[last] + serviceTime_[last] +
                                             tt.driveTime(pointId_[last], route.depotPointId());
        latest_[end] = route.depotReturnDeadline();
    }

    // Arcs of the precedence graph, weighted by their slack.
    std::vector<std::vector<std::pair<int, int>>> successors(nbNodes_);
    bool negativeSlack = false;
    auto addArc = [&](int from, int to, int travel) {
        int slack = earliest_[to] - earliest_[from] - serviceTime_[from] - travel;
        if (slack < 0) {
            negativeSlack = true;
            slack = 0;
        }
        successors[from].push_back({to, slack});
    };
    for (int r : routes) {
        const Route& route = solution.route(r);
        for (int u = node(r, -1); u < node(r, route.size()); ++u) addArc(u, u + 1, tt.driveTime(pointId_[u], pointId_[u + 1]));
    }
    for (const Transfer& t : solution.transfers()) {
        if (routeOffset_[t.routeBefore] < 0) continue;  // another set of linked routes
        addArc(node(t.routeBefore, t.posStopBefore), node(t.routeAfter, t.posStopAfter), t.ptTravelTime);
    }
    if (negativeSlack) {
        std::cerr << "[LinkedRoutesTiming] earliest times violate a precedence -- ltwDarp not up to date?\n";
    }

    // Topological order (Kahn): the graph of a feasible solution is acyclic.
    std::vector<int> inDegree(nbNodes_, 0);
    for (const auto& arcs : successors)
        for (const auto& [to, slack] : arcs) ++inDegree[to];
    std::vector<int> order;
    order.reserve(nbNodes_);
    for (int u = 0; u < nbNodes_; ++u)
        if (inDegree[u] == 0) order.push_back(u);
    for (size_t k = 0; k < order.size(); ++k) {
        for (const auto& [to, slack] : successors[order[k]])
            if (--inDegree[to] == 0) order.push_back(to);
    }
    if (static_cast<int>(order.size()) != nbNodes_) {
        std::cerr << "[LinkedRoutesTiming] cycle in the precedence graph -- inconsistent transfers\n";
    }

    // ST (Section 8): from every node, shortest paths in the DAG, visiting
    // the nodes in topological order from that node's own rank onwards.
    // Then F_u = min over u and its successors i of ST_{u,i} + l_i - h_i.
    slackTime_.assign(static_cast<size_t>(nbNodes_) * nbNodes_, kUnreachable);
    forwardSlack_.assign(nbNodes_, 0);
    for (size_t rank = 0; rank < order.size(); ++rank) {
        int u = order[rank];
        int* row = &slackTime_[static_cast<size_t>(u) * nbNodes_];
        row[u] = 0;
        int slack = latest_[u] - earliest_[u];
        for (size_t k = rank; k < order.size(); ++k) {
            int from = order[k];
            if (row[from] == kUnreachable) continue;
            slack = std::min(slack, row[from] + latest_[from] - earliest_[from]);
            for (const auto& [to, arcSlack] : successors[from]) row[to] = std::min(row[to], row[from] + arcSlack);
        }
        forwardSlack_[u] = slack;
    }
}
