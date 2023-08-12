#pragma once

#include <string>
#include <vector>

using namespace std;

class TransportationLine
{
public:
	string name;
	int id; //Line number in vector of lines
	bool bCircular;

	vector<int> stops; //indexes of Transportation stops in TransportationSystem.stops belogning to this line

	TransportationLine(string _name, bool bCircular = false);

	void AddStop(int stopIndex);
};

