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
    int percentRequestsRemovedMin() const noexcept { return percentRequestsRemovedMin_; }
    int percentRequestsRemovedMax() const noexcept { return percentRequestsRemovedMax_; }
    int walkThreshold() const noexcept { return walkThreshold_; }
    int minimumTimeInTransit() const noexcept { return minimumTimeInTransit_; }
    int waitingTime() const noexcept { return waitingTime_; }

    float recordToRecordValue() const noexcept { return recordToRecordValue_; }
    int updateIteration() const noexcept { return updateIteration_; }
    float rateRRT() const noexcept { return rateRRT_; }
    int upperBoundRRT() const noexcept { return upperBoundRRT_; }
    int lowerBoundRRT() const noexcept { return lowerBoundRRT_; }
    int nbPtOptions() const noexcept { return nbPtOptions_; }

private:
    // --- Search parameters (searchConfigFile) ---
    int percentRequestsRemovedMin_ = 0;
    int percentRequestsRemovedMax_ = 0;
    // Acceptance threshold for record-to-record travel (< 1).
    float recordToRecordValue_ = 0.f;
    // recordToRecordValue_ is updated every updateIteration_ feasible iterations.
    int updateIteration_ = 0;
    // Number of non-improving iterations that triggers an increase of recordToRecordValue_.
    int upperBoundRRT_ = 0;
    // Number of improving iterations that triggers a decrease of recordToRecordValue_.
    int lowerBoundRRT_ = 0;
    float rateRRT_ = 0.f;
    // Maximum number of PT options kept per request: stop pairs for the
    // 2-vehicle pattern, vehicle-side stops for the walk pattern (see
    // Instance::precomputeAccessOptions).
    int nbPtOptions_ = 0;

    // --- Instance-related runtime parameters (instanceConfigFile) ---
    int timeLimit_ = 0;
    int walkThreshold_ = 0;
    int minimumTimeInTransit_ = 0;
    int waitingTime_ = 0;
};

std::ostream& operator<<(std::ostream& os, const ConfLNS& conf);
