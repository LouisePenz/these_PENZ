#include "ConfLNS.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>

namespace {

// Parses a simple "key: value" file into a map, trimming whitespace around
// both the key and the value. Returns an empty map (and prints nothing) if
// the file is missing a given key -- callers decide what to do about that.
std::map<std::string, std::string> parseKeyValueFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Impossible d'ouvrir le fichier de configuration : " << filename << std::endl;
        exit(1);
    }

    std::map<std::string, std::string> config;
    std::string line;
    while (std::getline(file, line)) {
        size_t delimiterPos = line.find(':');
        if (delimiterPos == std::string::npos) continue;

        std::string key = line.substr(0, delimiterPos);
        std::string value = line.substr(delimiterPos + 1);

        key.erase(key.find_last_not_of(" \t") + 1);
        key.erase(0, key.find_first_not_of(" \t"));
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        config[key] = value;
    }
    return config;
}

}  // namespace

ConfLNS::ConfLNS(const std::string& searchConfigFile, const std::string& instanceConfigFile) {
    std::map<std::string, std::string> searchConfig = parseKeyValueFile(searchConfigFile);
    nbRequestsRemovedMin_ = std::stoi(searchConfig.at("nb_requests_removed_min"));
    nbRequestsRemovedMax_ = std::stoi(searchConfig.at("nb_requests_removed_max"));
    rrtStartThreshold_ = std::stod(searchConfig.at("rrt_start_threshold"));
    if (searchConfig.count("rrt_end_threshold"))
        rrtEndThreshold_ = std::stod(searchConfig.at("rrt_end_threshold"));
    nbPtOptions_ = std::stoi(searchConfig.at("nb_pt_options"));
    if (searchConfig.count("worst_removal_randomness"))
        worstRemovalRandomness_ = std::stod(searchConfig.at("worst_removal_randomness"));
    if (searchConfig.count("related_removal_randomness"))
        relatedRemovalRandomness_ = std::stod(searchConfig.at("related_removal_randomness"));
    if (searchConfig.count("set_partitioning_pool_ratio"))
        setPartitioningPoolRatio_ = std::stoi(searchConfig.at("set_partitioning_pool_ratio"));

    std::map<std::string, std::string> instanceConfig = parseKeyValueFile(instanceConfigFile);
    waitingTime_ = std::stoi(instanceConfig.at("waiting_time"));
    minimumTimeInTransit_ = std::stoi(instanceConfig.at("minimum_time_in_transport_public"));
    walkThreshold_ = std::stoi(instanceConfig.at("threshold_to_walk"));
    timeLimit_ = std::stoi(instanceConfig.at("time_limit"));
    // No iteration limit if the key is absent or <= 0: the time limit alone stops the LNS.
    iterationLimit_ = instanceConfig.count("iteration_limit") ? std::stoi(instanceConfig.at("iteration_limit")) : 0;
    if (iterationLimit_ <= 0) iterationLimit_ = std::numeric_limits<int>::max();
}

std::string ConfLNS::toString() const {
    std::string s = " The LNS is executed until " + std::to_string(timeLimit_);
    s += " seconds or " + std::to_string(iterationLimit_) + " iterations.\n";
    s += "At each iteration, between " + std::to_string(nbRequestsRemovedMin_);
    s += " and " + std::to_string(nbRequestsRemovedMax_) + " requests are removed.\n";
    return s;
}

std::ostream& operator<<(std::ostream& os, const ConfLNS& conf) {
    os << conf.toString();
    return os;
}
