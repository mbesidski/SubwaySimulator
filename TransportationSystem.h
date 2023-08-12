#pragma once

#include <vector>
#include <map>

#include "TransportationStop.h"
#include "TransportationLine.h"
#include "TransportationIntersection.h"

using namespace std;

class TransportationSystem
{
public:

	vector<TransportationStop> stops;
	vector<TransportationLine> lines;
	vector<TransportationIntersection> intersections;

	map<pair<int, int>, float> DistanceBetweenStops; //defines distance in kilometers between stop pairs identified by indexes
	map<pair<int, int>, int> TimeBetweenStops; //defines distance between stops pairs identified by indexes in terms of time
	map<pair<int, int>, vector<int>> PathBetweenStops; //defines the path between stops pairs identified by indexes

	int AddLine(TransportationLine);

	int AddStop(TransportationStop);

	TransportationSystem();

	void CalculateStopDistances();
	void CalculateStopTimes(float acceleration, float maxSpeed);
	void CalculateIntersections();
	void CalculatePaths(int transferTime, int addlTransferTime);
};

