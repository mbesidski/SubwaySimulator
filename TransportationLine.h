#pragma once

#include <string>
#include <vector>
#include <map>
#include "TransportationLineColor.h"
#include "TransportationLineSegment.h"

using namespace std;

class TransportationLine
{
public:
	int id; //Line number in vector of lines
	bool bCircular;
	TransportationLineColor color;

	float length;
	
	float fullLength;
	int vehiclesPerTunnel; //vehicles per tunnel (or lane)

	int numTunnels; //how many tunnels one way tunnels (lanes if were talking about cars) will be needed
	int numVehicles; //overall total number of vehicles;

	vector<int> stops; //indexes of Transportation stops in TransportationSystem.stops belogning to this line

	map<pair<int, int>, TransportationLineSegment> segments; //for van lines, number of tunnels is calculated per segment and so is the number of vehicles

	TransportationLine(TransportationLineColor _color, bool bCircular = false);

	void AddStop(int stopIndex);
};

