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
	map<pair<int, int>, int> TrafficMap; //maps pair of stops to rush hour traffic between them

	int adjustedStopTime; //the stop time depending on whether it is a train system or underground highway system

	int tunnelCapacity; //how many people can move through this tunnel per direction per hour

	int totalVehicles; // number of vehicles in the entire system
	float totalTunnelLength; //overall tunnel length in km


	int AddLine(TransportationLine);

	int AddStop(TransportationStop);

	int FindClosestTransportartionStop(TransportationStopCoordinates coord, float proximity);

	wstring GetStopLabel(int stop_idx);

	wstring GetStopInfo(int stop_idx, int originStop);

	wstring GetLineInfo(int line_idx, TransportationStopCoordinates coords);

	wstring GetTotals();
	
	TransportationSystem();

	void CalculateStopDistances();
	void CalculateStopTimes();
	void CalculateIntersections();
	void CalculatePaths();
	void CalculateSuportingInfo();
	void CalculatePopulationDistribution();
	void CalculateTravelMap();
	void CalculateTrafficData();
	void CalculateLineData();
	void CalculateTunnelInfo();
	void CalculateTunnelNum();
	void CalculateSystemTotals();

};

