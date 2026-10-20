#include "Route.h"

#include <algorithm>
#include <cassert>

Route::Route(int vehicleId, int capacity, int depotPointId, int depotDeparture, int depotReturnDeadline)
    : vehicleId_(vehicleId),
      capacity_(capacity),
      depotPointId_(depotPointId),
      depotDeparture_(depotDeparture),
      depotReturnDeadline_(depotReturnDeadline) {}

void Route::insertStop(int position, RouteStop stop) {
    assert(position >= 0 && position <= size());
    stops_.insert(stops_.begin() + position, stop);
}

void Route::removeStop(int position) {
    assert(position >= 0 && position < size());
    stops_.erase(stops_.begin() + position);
}

void Route::recomputeLoad() {
    loadAfterStop_.resize(stops_.size());
    int running = 0;
    for (int i = 0; i < size(); ++i) {
        running += stops_[i].loadDelta;
        loadAfterStop_[i] = running;
    }
}

int Route::loadAfter(int position) const { return loadAfterStop_[position]; }

bool Route::hasFreeCapacity(int posFrom, int posTo, int extraLoad) const {
    for (int i = posFrom; i <= posTo; ++i) {
        if (loadAfterStop_[i] + extraLoad > capacity_) return false;
    }
    return true;
}
