#pragma once
#include <vector>
using namespace std;

class DijkstraPath
{
public:
	int origin;
	int destination;
	int time;
	float distance;
	vector<int> stops;
	int travelers;
	int intersections;

	DijkstraPath();
	DijkstraPath(int _origin, int _destination, int _time, vector<int> _stops);
};

