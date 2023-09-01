#include "TransportationLineSegment.h"

TransportationLineSegment::TransportationLineSegment()
{
	origin_stop_idx = -1;
	destination_stop_idx = -1;

	num_tunnels = -1;
	num_approach_tunnels = -1;
	addl_vehicles = -1;
	line_vehicles = -1;

	length = -1;
	total_length = -1;
}
