#pragma once

#include <chrono>

// A small time utility replacing the old Time.h/float_time_point -- nothing
// project-specific here, just std::chrono under a shorter name.
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

inline TimePoint now() { return Clock::now(); }

inline double secondsSince(TimePoint start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
