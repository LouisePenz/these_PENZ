#include "Request.h"

#include <iostream>

Request::Request(int pickupId, TimeWindow pickupWindow, int deliveryId, TimeWindow deliveryWindow,
                  int quantity, int maxRideTime, int loadingTime)
    : pickupId_(pickupId),
      pickupWindow_(pickupWindow),
      deliveryId_(deliveryId),
      deliveryWindow_(deliveryWindow),
      quantity_(quantity),
      maxRideTime_(maxRideTime),
      loadingTime_(loadingTime) {}

std::string Request::toString() const {
    std::string str;
    str += std::to_string(pickupId_) + "[" + std::to_string(pickupWindow_.lowerBound) + "," +
           std::to_string(pickupWindow_.upperBound) + "] ";
    str += std::to_string(deliveryId_) + "[" + std::to_string(deliveryWindow_.lowerBound) + "," +
           std::to_string(deliveryWindow_.upperBound) + "] ";
    str += std::to_string(maxRideTime_) + "\n";
    return str;
}

void Request::print() const {
    std::cout << "   Pickup at index " << pickupId_ << ", time window [" << pickupWindow_.lowerBound
              << ";" << pickupWindow_.upperBound << "]\n";
    std::cout << "   Delivery at index " << deliveryId_ << ", time window [" << deliveryWindow_.lowerBound
              << ";" << deliveryWindow_.upperBound << "]\n";
    std::cout << "   Service time " << loadingTime_ << "\n";
    std::cout << "   Quantity " << quantity_ << "\n";
    std::cout << "   Maximum riding time " << maxRideTime_ << "\n";

    if (!ptAccess_.available()) {
        std::cout << "   The request cannot take public transportation\n";
    } else {
        std::cout << "   Stops before are ";
        for (auto s : ptAccess_.stopsBefore()) std::cout << s << " ";
        std::cout << "\n   Stops after are ";
        for (auto s : ptAccess_.stopsAfter()) std::cout << s << " ";
        std::cout << "\n";
        auto before = ptAccess_.aggregateWindowBefore();
        std::cout << "   For stops before, the largest time window is [" << before.lowerBound << ";"
                  << before.upperBound << "]\n";
        for (auto s : ptAccess_.stopsBefore()) {
            auto w = ptAccess_.windowForStopBefore(s);
            std::cout << "      " << s << ": [" << w.lowerBound << ";" << w.upperBound << "]\n";
        }
        auto after = ptAccess_.aggregateWindowAfter();
        std::cout << "   For stops after, the largest time window is [" << after.lowerBound << ";"
                  << after.upperBound << "]\n";
        for (auto s : ptAccess_.stopsAfter()) {
            auto w = ptAccess_.windowForStopAfter(s);
            std::cout << "      " << s << ": [" << w.lowerBound << ";" << w.upperBound << "]\n";
        }
    }

    switch (walkAccess_.side()) {
        case WalkSide::None:
            std::cout << "   The request cannot walk\n";
            break;
        case WalkSide::AtPickup:
        case WalkSide::AtDelivery: {
            std::cout << "   The request can walk on the "
                      << (walkAccess_.side() == WalkSide::AtPickup ? "first" : "last") << " leg\n";
            std::cout << "   Its walk time is " << walkAccess_.walkTime() << "\n";
            std::cout << "   The nearest stop is " << walkAccess_.nearestStop() << "\n";
            std::cout << "   The complementary stops are ";
            for (auto s : walkAccess_.complementaryStops()) std::cout << s << " ";
            std::cout << "\n";
            auto w = walkAccess_.aggregateWindow();
            std::cout << "   The largest time window is [" << w.lowerBound << ";" << w.upperBound << "]\n";
            for (auto s : walkAccess_.complementaryStops()) {
                auto sw = walkAccess_.windowForStop(s);
                std::cout << "      " << s << ": [" << sw.lowerBound << ";" << sw.upperBound << "]\n";
            }
            break;
        }
    }
}
