#include "TransportationIntersection.h"
#include <set>

TransportationIntersection::TransportationIntersection(TransportationStopCoordinates _coords, vector<int> _stops) : coords(_coords)
{
	stops = _stops;
}

void TransportationIntersection::Merge(TransportationIntersection& intersection)
{
	set<int> new_stops;
	new_stops.insert(stops.begin(), stops.end());
	new_stops.insert(intersection.stops.begin(), intersection.stops.end());
	stops.clear();
	stops.resize(new_stops.size());
	copy(new_stops.begin(), new_stops.end(), stops.begin());
}
