#pragma once

#include <algorithm>
#include <vector>

// A minimal 2-objective Pareto front (both objectives minimized): keeps only
// points not weakly dominated by another point already in the front. add()
// is O(current front size); fine given the small candidate counts here (a
// handful of routes/stops, not thousands).
template <typename Payload>
class ParetoFront2D {
public:
    struct Point {
        Payload payload;
        int cost;
        int secondary;
    };

    void add(Payload payload, int cost, int secondary) {
        for (const auto& p : points_) {
            if (p.cost <= cost && p.secondary <= secondary) return;  // dominated by an existing point, discard
        }
        points_.erase(std::remove_if(points_.begin(), points_.end(),
                                      [&](const Point& p) { return cost <= p.cost && secondary <= p.secondary; }),
                      points_.end());
        points_.push_back({std::move(payload), cost, secondary});
    }

    void merge(const ParetoFront2D& other) {
        for (const auto& p : other.points_) add(p.payload, p.cost, p.secondary);
    }

    const std::vector<Point>& points() const noexcept { return points_; }
    bool empty() const noexcept { return points_.empty(); }

private:
    std::vector<Point> points_;
};
