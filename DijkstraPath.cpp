#include "DijkstraPath.h"

DijkstraPath::DijkstraPath(int _origin, int _destination, int _time, vector<int> _stops)
{
	origin = _origin;
	destination = _destination;
	time = _time;
	
	stops = _stops;
}

DijkstraPath::DijkstraPath()
{
	origin = -1;
	destination = -1;
	time = -1;
	stops.clear();
}
