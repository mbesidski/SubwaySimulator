#include "Utils.h"
#include <cmath>

#include "TransportationSystemAssumptions.h"

float Utils::CalculateDistance(TransportationStopCoordinates coord1, TransportationStopCoordinates coord2)
{
	return std::sqrt(pow((coord2.x - coord1.x), 2) + pow((coord2.y - coord1.y), 2));
}

//Distance in km, Acceleration in m/s^2, Speed in m/s, Time in seconds
float Utils::CalculateTime(float acceleration, float maxSpeed, float distance, float maxAccelerationDistance)
{
	if (TransportationSystemAssumptions::isTrainLine)
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
	else
		return distance / maxSpeed;

}

wstring Utils::FormatWithCommas(long long int f)
{

	wstring num_str = to_wstring(f);
	wstring res = L"";

	int digitCounter = 0;

	for (int i = num_str.size() - 1; i >= 0; i--)
	{
		if (digitCounter == 3)
		{
			res += L",";
			digitCounter = 0;
		}

		res += num_str[i];
		digitCounter++;
	}

	reverse(res.begin(), res.end());

	return res;

}