#include "TravelTimeProvider.h"

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
