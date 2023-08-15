#pragma once

#include <string>
#include <vector>
#include "TransportationLineColor.h"

using namespace std;

class TransportationLine
{
public:
	int id; //Line number in vector of lines
	bool bCircular;
	TransportationLineColor color;

	vector<int> stops; //indexes of Transportation stops in TransportationSystem.stops belogning to this line

	TransportationLine(TransportationLineColor _color, bool bCircular = false);

	void AddStop(int stopIndex);
};

