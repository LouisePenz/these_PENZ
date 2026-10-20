#include "Route.h"

#include <algorithm>
#include <cassert>

Route::Route(int vehicleId, int capacity, int depotPointId, int depotDeparture, int depotReturnDeadline)
    : vehicleId_(vehicleId),
      capacity_(capacity),
      depotPointId_(depotPointId),
      depotDeparture_(depotDeparture),
      depotReturnDeadline_(depotReturnDeadline) {}

int Route::pointBefore(int position) const {
    return (position == 0) ? depotPointId_ : stops_[position - 1].pointId;
}

int Route::pointAfter(int position) const {
    return (position == size()) ? depotPointId_ : stops_[position].pointId;
}

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

void Route::recomputePdpWindows(const TravelTimeProvider& tt) {
    // Forward pass: ltwPdp = earliest feasible service time, ignoring ride-time-max.
    int prevPoint = depotPointId_;
    int prevTime = depotDeparture_;
    int prevService = 0;
    for (int i = 0; i < size(); ++i) {
        RouteStop& s = stops_[i];
        int arrival = prevTime + prevService + tt.driveTime(prevPoint, s.pointId);
        s.ltwPdp = std::max(arrival, s.windowLtw);
        prevPoint = s.pointId;
        prevTime = s.ltwPdp;
        prevService = s.serviceTime;
    }

    // Backward pass: utwPdp = latest feasible service time.
    int nextPoint = depotPointId_;
    int nextDeadline = depotReturnDeadline_;
    for (int i = size() - 1; i >= 0; --i) {
        RouteStop& s = stops_[i];
        int latest = nextDeadline - tt.driveTime(s.pointId, nextPoint);
        s.utwPdp = std::min(latest, s.windowUtw);
        nextPoint = s.pointId;
        nextDeadline = s.utwPdp - s.serviceTime;
    }
}

void Route::recomputeForwardTimeSlack(const TravelTimeProvider& tt) {
    int n = size();
    if (n == 0) return;

    stops_[n - 1].forwardTimeSlackPdp = stops_[n - 1].utwPdp - stops_[n - 1].ltwPdp;
    for (int i = n - 2; i >= 0; --i) {
        int wait = stops_[i + 1].ltwPdp -
                   (stops_[i].ltwPdp + stops_[i].serviceTime + tt.driveTime(stops_[i].pointId, stops_[i + 1].pointId));
        int ownSlack = stops_[i].utwPdp - stops_[i].ltwPdp;
        stops_[i].forwardTimeSlackPdp = std::min(ownSlack, wait + stops_[i + 1].forwardTimeSlackPdp);
    }
}

bool Route::pdpAdmitsInsertion(const TravelTimeProvider& tt, int position, int pointId, int serviceTime,
                                int windowLtw, int windowUtw, int* outArrival, int* outDelta) const {
    int predPoint = pointBefore(position);
    int predTime = (position == 0) ? depotDeparture_ : stops_[position - 1].ltwPdp;
    int predService = (position == 0) ? 0 : stops_[position - 1].serviceTime;

    int arrival = std::max(predTime + predService + tt.driveTime(predPoint, pointId), windowLtw);
    if (outArrival) *outArrival = arrival;
    if (arrival > windowUtw) {
        if (outDelta) *outDelta = 0;
        return false;  // can't even reach its own window
    }

    int departure = arrival + serviceTime;
    int succPoint = pointAfter(position);
    int originalArrivalAtSucc = (position == size()) ? (predTime + predService + tt.driveTime(predPoint, succPoint))
                                                       : stops_[position].ltwPdp;
    int newArrivalAtSucc = departure + tt.driveTime(pointId, succPoint);
    int delta = newArrivalAtSucc - originalArrivalAtSucc;
    if (delta < 0) delta = 0;
    if (outDelta) *outDelta = delta;
    if (delta == 0) return true;  // no added delay downstream at all

    int slackAtSucc = (position == size()) ? (depotReturnDeadline_ - originalArrivalAtSucc)
                                            : stops_[position].forwardTimeSlackPdp;
    return delta <= slackAtSucc;
}

bool Route::pdpAdmitsAdjacentInsertion(const TravelTimeProvider& tt, int position, int firstPointId,
                                        int firstServiceTime, int firstWindowLtw, int firstWindowUtw,
                                        int secondPointId, int secondServiceTime, int secondWindowLtw,
                                        int secondWindowUtw, int* outSecondArrival, int* outDelta) const {
    int predPoint = pointBefore(position);
    int predTime = (position == 0) ? depotDeparture_ : stops_[position - 1].ltwPdp;
    int predService = (position == 0) ? 0 : stops_[position - 1].serviceTime;

    int firstArrival = std::max(predTime + predService + tt.driveTime(predPoint, firstPointId), firstWindowLtw);
    if (firstArrival > firstWindowUtw) {
        if (outSecondArrival) *outSecondArrival = firstArrival;
        if (outDelta) *outDelta = 0;
        return false;
    }
    int firstDeparture = firstArrival + firstServiceTime;

    int secondArrival =
        std::max(firstDeparture + tt.driveTime(firstPointId, secondPointId), secondWindowLtw);
    if (outSecondArrival) *outSecondArrival = secondArrival;
    if (secondArrival > secondWindowUtw) {
        if (outDelta) *outDelta = 0;
        return false;
    }
    int secondDeparture = secondArrival + secondServiceTime;

    int succPoint = pointAfter(position);
    int originalArrivalAtSucc = (position == size()) ? (predTime + predService + tt.driveTime(predPoint, succPoint))
                                                       : stops_[position].ltwPdp;
    int newArrivalAtSucc = secondDeparture + tt.driveTime(secondPointId, succPoint);
    int delta = newArrivalAtSucc - originalArrivalAtSucc;
    if (delta < 0) delta = 0;
    if (outDelta) *outDelta = delta;
    if (delta == 0) return true;

    int slackAtSucc = (position == size()) ? (depotReturnDeadline_ - originalArrivalAtSucc)
                                            : stops_[position].forwardTimeSlackPdp;
    return delta <= slackAtSucc;
}
