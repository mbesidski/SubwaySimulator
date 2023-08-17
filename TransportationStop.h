#pragma once
#include <string>
#include "TransportationStopCoordinates.h"

using namespace std;

class TransportationStop
{
public:
	int id; //index in the arrray of stops in TrasportationSystem.stops
	int line_id; //index in the arrray of lines in TrasportationSystem.lines

	int ResidentSpaces;
	int WorkSpaces;

	TransportationStopCoordinates mapCoordinates;

	TransportationStop(TransportationStopCoordinates _coords, int _line_id);
};

