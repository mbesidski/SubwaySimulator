#pragma once
#include <vector>
using namespace std;

class DijkstraPath
{
public:
	int origin;
	int destination;
	int time;
	vector<int> stops;
	DijkstraPath();
	DijkstraPath(int _origin, int _destination, int _time, vector<int> _stops);
};

