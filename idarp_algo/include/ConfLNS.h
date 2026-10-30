#pragma once

#include <iosfwd>
#include <string>

enum class DestructionOperator {
    RandomRemoval,
    RelatedRemoval,
    WorstRemoval,
    RouteRemoval,
    ClusterRemoval,
    SeparateSelectionRemoval
};

enum class RepairOperator {
    RandomRepair,
    GreedyRepair,
    RegretRepair,
    ForceTransferRepair
};

// Loads LNS search parameters and instance-related runtime parameters from two
// key:value configuration files.
class ConfLNS {
public:
    ConfLNS(const std::string& searchConfigFile, const std::string& instanceConfigFile);

    std::string toString() const;

    int timeLimit() const noexcept { return timeLimit_; }
    int iterationLimit() const noexcept { return iterationLimit_; }
    int nbRequestsRemovedMin() const noexcept { return nbRequestsRemovedMin_; }
    int nbRequestsRemovedMax() const noexcept { return nbRequestsRemovedMax_; }
    int walkThreshold() const noexcept { return walkThreshold_; }
    int minimumTimeInTransit() const noexcept { return minimumTimeInTransit_; }
    int waitingTime() const noexcept { return waitingTime_; }

    double rrtStartThreshold() const noexcept { return rrtStartThreshold_; }
    double rrtEndThreshold() const noexcept { return rrtEndThreshold_; }
    int nbPtOptions() const noexcept { return nbPtOptions_; }
    double worstRemovalRandomness() const noexcept { return worstRemovalRandomness_; }
    double relatedRemovalRandomness() const noexcept { return relatedRemovalRandomness_; }
    int setPartitioningPoolRatio() const noexcept { return setPartitioningPoolRatio_; }

private:
    // --- Search parameters (searchConfigFile) ---
    // Number of requests removed at each iteration, drawn uniformly between
    // these bounds (capped by the number of requests of the instance).
    int nbRequestsRemovedMin_ = 0;
    int nbRequestsRemovedMax_ = 0;
    // Record-to-record travel threshold (relative gap to the best solution) at
    // the start and at the end of the search, linear in between (see LNS).
    double rrtStartThreshold_ = 0.0;
    // Optional key, default 0 ("fixed end", Santini, Ropke & Hvattum 2018).
    double rrtEndThreshold_ = 0.0;
    // Maximum number of PT options kept per request: stop pairs for the
    // 2-vehicle pattern, vehicle-side stops for the walk pattern (see
    // Instance::precomputeAccessOptions).
    int nbPtOptions_ = 0;
    // Randomness p >= 1 of WorstRemoval (see there). Optional key, default 3
    // (Ropke & Pisinger 2006).
    double worstRemovalRandomness_ = 3.0;
    // Randomness p >= 1 of RelatedRemoval, both measures (see there). Optional
    // key, default 6 (Ropke & Pisinger 2006).
    double relatedRemovalRandomness_ = 6.0;
    // The LNS runs the set partitioning each time its pool of routes reaches
    // this ratio / the number of requests routes, then empties it (see LNS).
    // Optional key, default 15000 * 30: 15000 routes for 30 requests, 4500
    // for 100.
    int setPartitioningPoolRatio_ = 15000 * 30;

    // --- Instance-related runtime parameters (instanceConfigFile) ---
    int timeLimit_ = 0;
    // The LNS also stops after this many iterations (see LNS::compute).
    int iterationLimit_ = 0;
    int walkThreshold_ = 0;
    int minimumTimeInTransit_ = 0;
    int waitingTime_ = 0;
};

std::ostream& operator<<(std::ostream& os, const ConfLNS& conf);
