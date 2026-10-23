#pragma once

// Links the two vehicle-served legs of a request that uses public transport
// with two different vehicles: pickup -> ... -> stopBefore on one route,
// stopAfter -> ... -> delivery on another. This is the only pattern that
// can't be folded into an ordinary single-leg pickup/delivery pair (walk and
// direct patterns don't need this at all -- see the discussion on why).
struct Transfer {
    int requestId;

    int routeBefore;
    int posPickup;
    int posStopBefore;

    int routeAfter;
    int posStopAfter;
    int posDelivery;

    // publicTransportTime(stopBefore, stopAfter) -- already includes the
    // average waiting time, fixed once the two stops are chosen.
    int ptTravelTime;
};
