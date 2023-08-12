#include "TransportationLine.h"

TransportationLine::TransportationLine(string _name, bool _bCircular)
{
	name = _name;
	id = -1;
	bCircular = _bCircular;

	stops.clear();
}

void TransportationLine::AddStop(int stopIndex)
{
	stops.push_back(stopIndex);
}
