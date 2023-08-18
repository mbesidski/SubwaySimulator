#include "TransportationSystem.h"
#include "TransportationSystemAssumptions.h"
#include "Utils.h"
#include "Dijkstra.h"

TransportationSystem::TransportationSystem()
{
	stops.clear();
	lines.clear();
}

int TransportationSystem::AddStop(TransportationStop stop)
{
	//check if we're trying to enter a duplicate point for the same line
	if (stops.size() > 0) //we have some stops already
	{
		TransportationStop prev_stop = stops[stops.size() - 1];
		if (prev_stop.line_id == stop.line_id) //same line
		{
			if (Utils::CalculateDistance(prev_stop.mapCoordinates, stop.mapCoordinates) < 0.05) // same stop location
				return -1; //do not add this stop - it is a duplicate
		}
	}

	stop.id = stops.size();
	stops.push_back(stop);

	lines[stop.line_id].stops.push_back(stop.id);

	return stop.id;
}

int TransportationSystem::AddLine(TransportationLine line)
{
	line.id = lines.size();
	lines.push_back(line);

	return line.id;
}
wstring TransportationSystem::GetStopInfo(int stop_idx)
{
	wstring label;

	if (IntersectionMap.find(stop_idx) != IntersectionMap.end())
	{
		label = L"Intersection " + to_wstring(IntersectionMap[stop_idx]) + L"\n";
		label += L"Stops ";
		wstring lines = L"Lines ";
		wstring spaces = L"";

		for (int i = 0; i < intersections[IntersectionMap[stop_idx]].stops.size(); i++)
		{
			if (i > 0)
			{
				label += L", ";
				lines += L", ";
			}
			
			label += to_wstring(intersections[IntersectionMap[stop_idx]].stops[i]);
			lines += to_wstring(stops[intersections[IntersectionMap[stop_idx]].stops[i]].line_id);

			spaces += L"Stop " + to_wstring(intersections[IntersectionMap[stop_idx]].stops[i]) + L"  ";
			spaces += L"ResidentSpaces " + to_wstring(stops[stop_idx].ResidentSpaces) + L"  ";
			spaces += L"WorkSpaces " + to_wstring(stops[stop_idx].WorkSpaces) + L"\n";
		}
		label += L"\n";
		label += lines + L"\n";
		label += spaces;
	}
	else
	{
		label = L"Stop " + to_wstring(stop_idx) + L"\n";
		label += L"Line " + to_wstring(stops[stop_idx].line_id) + L"\n";

		label += L"ResidentSpaces " + to_wstring(stops[stop_idx].ResidentSpaces) + L"\n";
		label += L"WorkSpaces " + to_wstring(stops[stop_idx].WorkSpaces) + L"\n";
	}

	label += L"Map Coordinates " + to_wstring(stops[stop_idx].mapCoordinates.x) + L", " + to_wstring(stops[stop_idx].mapCoordinates.y) + L"\n";

	return label;
}

wstring TransportationSystem::GetStopLabel(int stop_idx)
{
	wstring label;

	if (IntersectionMap.find(stop_idx) != IntersectionMap.end()) //this stop is a part of an intersection
	{
		/*for (int i = 0; i < intersections[IntersectionMap[stop_idx]].stops.size(); i++)
		{
			label += to_string(intersections[IntersectionMap[stop_idx]].stops[i]);
			if (i < intersections[IntersectionMap[stop_idx]].stops.size() - 1)
				label += ",";
		}*/
		label = L"i" + to_wstring(IntersectionMap[stop_idx]);
	}
	else
		label = to_wstring(stop_idx);

	return label;
}

void TransportationSystem::CalculateStopDistances()
{
	for (TransportationLine l : lines)
	{
		for (int i = 0; i < l.stops.size() - 1; i++)
		{
			TransportationStop curr_stop = stops[l.stops[i]];
			TransportationStop next_stop = stops[l.stops[i + 1]];
			float distance = Utils::CalculateDistance(curr_stop.mapCoordinates, next_stop.mapCoordinates);
			DistanceBetweenStops[{curr_stop.id, next_stop.id}] = distance;
			DistanceBetweenStops[{next_stop.id, curr_stop.id}] = distance;
		}

		if (l.bCircular)
		{
			TransportationStop curr_stop = stops[l.stops[0]];
			TransportationStop next_stop = stops[l.stops[l.stops.size()-1]];
			float distance = Utils::CalculateDistance(curr_stop.mapCoordinates, next_stop.mapCoordinates);
			DistanceBetweenStops[{curr_stop.id, next_stop.id}] = distance;
			DistanceBetweenStops[{next_stop.id, curr_stop.id}] = distance;
		}
	}

}

void TransportationSystem::CalculateStopTimes()
{
	float Tm = TransportationSystemAssumptions::maxSpeed / TransportationSystemAssumptions::acceleration;
	float maxAccelerationDistance = (TransportationSystemAssumptions::acceleration * pow(Tm, 2)) / 2;

	for (auto d : DistanceBetweenStops)
	{
		float distance = d.second;
		float time = Utils::CalculateTime(TransportationSystemAssumptions::acceleration, TransportationSystemAssumptions::maxSpeed, distance * 1000, maxAccelerationDistance);
		TimeBetweenStops[d.first] = round(time);
	}
}

void TransportationSystem::CalculateIntersections()
{
	for (TransportationLine line : lines)
	{
		for (int stop_idx : line.stops)
		{
			for (TransportationLine other_line : lines)
			{
				if (other_line.id == line.id)
					continue;

				for (int other_stop_idx : other_line.stops)
				{
					if (Utils::CalculateDistance(stops[stop_idx].mapCoordinates, stops[other_stop_idx].mapCoordinates) < TransportationSystemAssumptions::proximity_distance)
						intersections.push_back({ stops[stop_idx].mapCoordinates, {stop_idx, other_stop_idx} });
				}
			}
		}
	}

	bool bFound = true;
	while (bFound)
	{
		bFound = false;
		for (int i = 0; i < intersections.size(); i++)
		{
			for (int j = i + 1; j < intersections.size(); j++)
			{
				if (Utils::CalculateDistance(intersections[i].coords, intersections[j].coords) < TransportationSystemAssumptions::proximity_distance)
				{
					bFound = true;
					intersections[i].Merge(intersections[j]);
					intersections.erase(intersections.begin() + j);
					continue;
				}
			}
		}
	}

	for (int i = 0; i < intersections.size(); i++)
	{
		TransportationIntersection intersection = intersections[i];
		for (int stop_idx : intersection.stops)
			IntersectionMap[stop_idx] = i;
	}
}

void TransportationSystem::CalculatePaths()
{
	vector<vector<int>> adjacency_matrix;
	adjacency_matrix.resize(stops.size(), vector<int>(stops.size(), 0));

	for (auto iter = TimeBetweenStops.begin(); iter != TimeBetweenStops.end(); iter++)
		adjacency_matrix[iter->first.first][iter->first.second] = TimeBetweenStops[{iter->first.first, iter->first.second}];

	for (int i = 0; i < intersections.size(); i++)
	{
		int transfer_penalty = 
			TransportationSystemAssumptions::transferTime + 
			TransportationSystemAssumptions::addlTransferTime * (intersections[i].stops.size() - 2) / 2.0;

		for (int j = 0; j < intersections[i].stops.size(); j++)
		{
			for (int k = 0; k < intersections[i].stops.size(); k++)
			{
				if (j != k) 
					adjacency_matrix[intersections[i].stops[j]][intersections[i].stops[k]] = transfer_penalty;
			}
		}
	}

	for (TransportationStop stop : stops)
	{
		vector<DijkstraPath> paths = dijkstra(adjacency_matrix, stop.id);
		
		for (auto path : paths)
			PathBetweenStops[{path.origin, path.destination}] = path;
	}
}

void TransportationSystem::CalculatePopulationDistribution()
{
	map<int, float> distanceToCenter;
	for (int i = 0; i < stops.size(); i++)
	{
		float dist = Utils::CalculateDistance(stops[i].mapCoordinates, TransportationSystemAssumptions::MapCenter());
		distanceToCenter[stops[i].id] = dist;
	}
	float maxDist = distanceToCenter[0];
	for (int i = 1; i < distanceToCenter.size(); i++)
	{
		if (distanceToCenter[i] > maxDist)
		{
			maxDist = distanceToCenter[i];
		}
	}
	map<int, float> populationproportions;
	float populationproportionsum = 0.0;
	map<int, float> officeproportions;
	float officeproportionsum = 0.0;

	for (int i = 0; i < stops.size(); i++)
	{
		float numStops = 1.0;
		if (IntersectionMap.find(stops[i].id) != IntersectionMap.end())
			numStops = intersections[IntersectionMap[stops[i].id]].stops.size();

		float officeproportion = (1.0 - (distanceToCenter[i] / maxDist) * .9)/numStops;

		officeproportionsum = officeproportionsum + officeproportion;
		float populationproportion = (0.1 + (distanceToCenter[i] / maxDist) * .9)/numStops;
		populationproportionsum = populationproportionsum + populationproportion;
		populationproportions[stops[i].id] = populationproportion;
		officeproportions[stops[i].id] = officeproportion;
	}

	float popRatio = TransportationSystemAssumptions::CommutingPopulation / populationproportionsum;
	float officeRatio = TransportationSystemAssumptions::CommutingPopulation / officeproportionsum;
	for (int i = 0; i < stops.size(); i++)
	{
		
		stops[i].WorkSpaces = officeRatio * officeproportions[i];
		stops[i].ResidentSpaces = popRatio * populationproportions[i];

	}

}

void TransportationSystem::CalculateSuportingInfo()
{
	CalculateStopDistances();
	CalculateStopTimes();
	CalculateIntersections();
	CalculatePaths();
	CalculatePopulationDistribution();
}

//proximity is in kilometers from mouse click point
int TransportationSystem::FindClosestTransportartionStop(TransportationStopCoordinates coord, float proximity)
{
	for (TransportationStop stop : stops)
	{
		if (Utils::CalculateDistance(stop.mapCoordinates, coord) < proximity)
			return stop.id;
	}

	return -1;
}