#pragma once

#include <vector>
#include <map>

#include "TransportationStop.h"
#include "TransportationLine.h"
#include "TransportationIntersection.h"
#include "DijkstraPath.h"

using namespace std;

class TransportationSystem
{
public:

	vector<TransportationStop> stops;
	vector<TransportationLine> lines;
	vector<TransportationIntersection> intersections;

	map<pair<int, int>, float> DistanceBetweenStops; //defines distance in kilometers between stop pairs identified by indexes
	map<pair<int, int>, int> TimeBetweenStops; //defines distance between stops pairs identified by indexes in terms of time
	map<pair<int, int>, DijkstraPath> PathBetweenStops; //defines the path between stops pairs identified by indexes
	map<int, int> IntersectionMap; //stop_idx to intersection_idx

	int AddLine(TransportationLine);

	int AddStop(TransportationStop);

	int FindClosestTransportartionStop(TransportationStopCoordinates coord, float proximity);

	wstring GetStopLabel(int stop_idx);

	wstring GetStopInfo(int stop_idx, int originStop);

	TransportationSystem();

	void CalculateStopDistances();
	void CalculateStopTimes();
	void CalculateIntersections();
	void CalculatePaths();
	void CalculateSuportingInfo();
	void CalculatePopulationDistribution();
	void CalculateTravelMap();
};

