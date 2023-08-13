#include "Utils.h"
#include <cmath>

float Utils::CalculateDistance(TransportationStopCoordinates coord1, TransportationStopCoordinates coord2)
{
	return std::sqrt(pow((coord2.x - coord1.x), 2) + pow((coord2.y - coord1.y), 2));
}

//Distance in km, Acceleration in m/s^2, Speed in m/s, Time in seconds
float Utils::CalculateTime(float acceleration, float maxSpeed, float distance, float maxAccelerationDistance)
{

	if (distance / 2 < maxAccelerationDistance)
	{
		float time = 2 * (sqrt(2 * (distance / 2) / acceleration));
		return time;
	}
	else
	{
		float time = 2 * (maxSpeed / acceleration);
		float remainingDistance = distance - 2 * maxAccelerationDistance;
		time = time + (remainingDistance / maxSpeed);
		return time;
	}
}