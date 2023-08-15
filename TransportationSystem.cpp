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

string TransportationSystem::GetStopLabel(int stop_idx)
{
	string label;

	if (IntersectionMap.find(stop_idx) != IntersectionMap.end()) //this stop is a part of an intersection
	{
		for (int i = 0; i < intersections[IntersectionMap[stop_idx]].stops.size(); i++)
		{
			label += to_string(intersections[IntersectionMap[stop_idx]].stops[i]);
			if (i < intersections[IntersectionMap[stop_idx]].stops.size() - 1)
				label += ",";
		}
	}
	else
		label = to_string(stop_idx);

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
			TransportationSystemAssumptions::addlTransferTime * (intersections[i].stops.size() - 2);

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

void TransportationSystem::CalculateSuportingInfo()
{
	CalculateStopDistances();
	CalculateStopTimes();
	CalculateIntersections();
	CalculatePaths();

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