#include "Request.h"

Request::Request(int pickupId, TimeWindow pickupWindow, int deliveryId, TimeWindow deliveryWindow,
                  int quantity, int maxRideTime, int loadingTime)
    : pickupId_(pickupId),
      pickupWindow_(pickupWindow),
      deliveryId_(deliveryId),
      deliveryWindow_(deliveryWindow),
      quantity_(quantity),
      maxRideTime_(maxRideTime),
      loadingTime_(loadingTime) {}
