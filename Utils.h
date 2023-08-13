#pragma once

#include "TransportationStopCoordinates.h"

class Utils
{
public:
	static float CalculateDistance(TransportationStopCoordinates coord1, TransportationStopCoordinates coord2);
	static float CalculateTime(float acceleration, float maxSpeed, float distance, float maxAccelerationDistance);

};

