#include "TransportationStop.h"

TransportationStop::TransportationStop(TransportationStopCoordinates _coords, int _line_id) : mapCoordinates(_coords.x, _coords.y)
{
	id = -1;
	line_id = _line_id;
}
