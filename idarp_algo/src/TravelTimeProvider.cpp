#include "TravelTimeProvider.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>

TravelTimeProvider::TravelTimeProvider(const PointRegistry& registry)
    : registry_(registry),
      nbVertices_(registry.nbVertices()),
      nbStops_(registry.nbStops()),
      firstPtStop_(registry.fromPtLocalIndex(0)) {}

void TravelTimeProvider::loadMatrix(int rows, int cols, std::vector<int>& matrix, const std::string& filename) {
    std::ifstream in(filename, std::ios::in);
    if (!in.is_open()) {
        std::cerr << "The file of your problem cannot be open: " << filename << std::endl;
        exit(1);
    }
    matrix.clear();
    matrix.reserve(static_cast<size_t>(rows) * cols);
    int value;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            in >> value;
            matrix.push_back(value);
        }
    }
}

void TravelTimeProvider::loadDriveTime(const std::string& filename) {
    int n = registry_.nbVertices();
    loadMatrix(n, n, driveTime_, filename);
}

void TravelTimeProvider::loadWalkTime(const std::string& filename) {
    int n = registry_.nbVertices();
    loadMatrix(n, n, walkTime_, filename);
}

void TravelTimeProvider::loadPublicTransportTime(const std::string& filename) {
    int nbStops = registry_.nbStops();
    loadMatrix(nbStops, nbStops, publicTransportTime_, filename);
}

void TravelTimeProvider::loadTramTimetable(const std::string& filename) {
    std::ifstream in(filename, std::ios::in);
    if (!in.is_open()) {
        std::cerr << "The file of your problem cannot be open: " << filename << std::endl;
        exit(1);
    }
    // Every (departure, arrival) of a direct run, per pair of stops.
    std::vector<std::vector<std::pair<int, int>>> rides(static_cast<size_t>(nbStops_) * nbStops_);
    int nbRuns = 0;
    in >> nbRuns;
    for (int run = 0; run < nbRuns; ++run) {
        std::string label;
        int nbRunStops = 0;
        in >> label >> nbRunStops;
        std::vector<int> stops(nbRunStops), times(nbRunStops);
        for (int k = 0; k < nbRunStops; ++k) in >> stops[k] >> times[k];
        for (int i = 0; i < nbRunStops; ++i)
            for (int j = i + 1; j < nbRunStops; ++j)
                if (stops[i] != stops[j])
                    rides[static_cast<size_t>(stops[i]) * nbStops_ + stops[j]].push_back({times[i], times[j]});
    }
    if (!in) {
        std::cerr << "Malformed tram timetable: " << filename << std::endl;
        exit(1);
    }

    directRuns_.assign(rides.size(), DirectRuns{});
    publicTransportTime_.assign(rides.size(), kNoTram);
    for (size_t pair = 0; pair < rides.size(); ++pair) {
        std::vector<std::pair<int, int>>& r = rides[pair];
        if (r.empty()) continue;
        // Pareto filter, latest departures first: keep a run only if it
        // arrives strictly earlier than every run leaving at the same time or later.
        std::sort(r.begin(), r.end(), [](const auto& x, const auto& y) {
            return x.first != y.first ? x.first > y.first : x.second < y.second;
        });
        DirectRuns& kept = directRuns_[pair];
        int bestArrival = kNoTram;
        for (const auto& [dep, arr] : r) {
            if (arr >= bestArrival) continue;
            bestArrival = arr;
            kept.dep.push_back(dep);
            kept.arr.push_back(arr);
            publicTransportTime_[pair] = std::min(publicTransportTime_[pair], arr - dep);
        }
        std::reverse(kept.dep.begin(), kept.dep.end());
        std::reverse(kept.arr.begin(), kept.arr.end());
    }
    for (int s = 0; s < nbStops_; ++s) publicTransportTime_[static_cast<size_t>(s) * nbStops_ + s] = 0;
    hasTimetable_ = true;
}

std::pair<int, int> TravelTimeProvider::earliestRun(int a, int b, int t, int depCap, int arrFloor) const {
    const DirectRuns& r = directRuns(a, b);
    auto end = std::upper_bound(r.dep.begin(), r.dep.end(), depCap) - r.dep.begin();
    auto first = std::max(std::lower_bound(r.dep.begin(), r.dep.end(), t) - r.dep.begin(),
                          std::lower_bound(r.arr.begin(), r.arr.end(), arrFloor) - r.arr.begin());
    return first < end ? std::make_pair(r.dep[first], r.arr[first]) : std::make_pair(kNoTram, kNoTram);
}

std::pair<int, int> TravelTimeProvider::latestRun(int a, int b, int t, int depCap, int arrFloor) const {
    const DirectRuns& r = directRuns(a, b);
    auto last = std::min(std::upper_bound(r.arr.begin(), r.arr.end(), t) - r.arr.begin(),
                         std::upper_bound(r.dep.begin(), r.dep.end(), depCap) - r.dep.begin()) - 1;
    auto first = std::lower_bound(r.arr.begin(), r.arr.end(), arrFloor) - r.arr.begin();
    return last >= first ? std::make_pair(r.dep[last], r.arr[last]) : std::make_pair(kNoTram, kNoTram);
}

int TravelTimeProvider::earliestArrival(int a, int b, int t, int depCap, int arrFloor) const {
    return earliestRun(a, b, t, depCap, arrFloor).second;
}

int TravelTimeProvider::latestDeparture(int a, int b, int t, int depCap, int arrFloor) const {
    int departure = latestRun(a, b, t, depCap, arrFloor).first;
    return departure == kNoTram ? -kNoTram : departure;
}
