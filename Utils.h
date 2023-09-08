#pragma once

#include "TransportationStopCoordinates.h"
#include <algorithm>
#include <iostream>
#include <string>

using namespace std;

class Utils
{
public:
	static float CalculateDistance(TransportationStopCoordinates coord1, TransportationStopCoordinates coord2);
	static float CalculateTime(float acceleration, float maxSpeed, float distance, float maxAccelerationDistance);
	static wstring FormatWithCommas(long long int f);

};

