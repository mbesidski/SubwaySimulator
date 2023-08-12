#pragma once
#include "TransportationStopCoordinates.h"
#include <vector>

using namespace std;

class TransportationIntersection
{
public:
	TransportationStopCoordinates coords;
	vector<int> stops;

	TransportationIntersection(TransportationStopCoordinates _coords, vector<int> _stops);
	void Merge(TransportationIntersection& intersection);
};

