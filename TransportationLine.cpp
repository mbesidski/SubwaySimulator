#include "TransportationLine.h"

TransportationLine::TransportationLine(TransportationLineColor _color, bool _bCircular) : color(_color.color, _color.name)
{
	id = -1;
	bCircular = _bCircular;

	stops.clear();
}

void TransportationLine::AddStop(int stopIndex)
{
	stops.push_back(stopIndex);
}
