#include "ConfLNS.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
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
    percentRequestsRemovedMin_ = std::stoi(searchConfig.at("pourcent_nb_requests_removed_min"));
    percentRequestsRemovedMax_ = std::stoi(searchConfig.at("pourcent_nb_requests_removed_max"));
    recordToRecordValue_ = std::stof(searchConfig.at("record_to_record_value"));
    upperBoundRRT_ = std::stoi(searchConfig.at("upper_bound_RRT"));
    lowerBoundRRT_ = std::stoi(searchConfig.at("lower_bound_RRT"));
    rateRRT_ = std::stof(searchConfig.at("rate_RRT"));
    updateIteration_ = std::stoi(searchConfig.at("update_iteration"));
    nbPtOptions_ = std::stoi(searchConfig.at("nb_pt_options"));

    std::map<std::string, std::string> instanceConfig = parseKeyValueFile(instanceConfigFile);
    waitingTime_ = std::stoi(instanceConfig.at("waiting_time"));
    minimumTimeInTransit_ = std::stoi(instanceConfig.at("minimum_time_in_transport_public"));
    walkThreshold_ = std::stoi(instanceConfig.at("threshold_to_walk"));
    timeLimit_ = std::stoi(instanceConfig.at("time_limit"));
}

std::string ConfLNS::toString() const {
    std::string s = " The LNS is executed until " + std::to_string(timeLimit_);
    s += " seconds.\n";
    s += "At each iteration, between " + std::to_string(percentRequestsRemovedMin_);
    s += " and " + std::to_string(percentRequestsRemovedMax_) + "% of requests are removed.\n";
    return s;
}

std::ostream& operator<<(std::ostream& os, const ConfLNS& conf) {
    os << conf.toString();
    return os;
}
