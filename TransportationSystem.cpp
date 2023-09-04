#include "TransportationSystem.h"
#include "TransportationSystemAssumptions.h"
#include "Utils.h"
#include "Dijkstra.h"

TransportationSystem::TransportationSystem()
{
	stops.clear();
	lines.clear();
	adjustedStopTime = -1;
	tunnelCapacity = -1;

	totalTunnelLength = -1;
	totalVehicles = -1;
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
wstring TransportationSystem::GetStopInfo(int stop_idx, int originStop)
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
			spaces += L"ResidentSpaces " + to_wstring(stops[intersections[IntersectionMap[stop_idx]].stops[i]].ResidentSpaces) + L"  ";
			spaces += L"WorkSpaces " + to_wstring(stops[intersections[IntersectionMap[stop_idx]].stops[i]].WorkSpaces) + L"  ";
			spaces += L"Coordinates " + to_wstring(stops[intersections[IntersectionMap[stop_idx]].stops[i]].mapCoordinates.x) + L", " + to_wstring(stops[intersections[IntersectionMap[stop_idx]].stops[i]].mapCoordinates.y) + L"  ";
			if (originStop >= 0)
			{
				spaces += L"Distance from " + to_wstring(originStop) + L" to " +
					to_wstring(intersections[IntersectionMap[stop_idx]].stops[i]) + L" = " +
					to_wstring(PathBetweenStops[{originStop, intersections[IntersectionMap[stop_idx]].stops[i]}].distance) + L"  ";

					spaces += L"Travelers from " + to_wstring(originStop) + L" to " +
					to_wstring(intersections[IntersectionMap[stop_idx]].stops[i]) + L" = " +
					to_wstring(PathBetweenStops[{originStop, intersections[IntersectionMap[stop_idx]].stops[i]}].travelers);
			}
			spaces += L"\n";
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
		
		if (originStop >= 0)
		{
			label += L"Distance between " + to_wstring(originStop) + L" and ";
			label += to_wstring(stop_idx) + L" = ";
			label += to_wstring(PathBetweenStops[{originStop, stop_idx}].distance) + L"\n";

			label += L"Travelers between " + to_wstring(originStop) + L" and ";
			label += to_wstring(stop_idx) + L" = ";
			label += to_wstring(PathBetweenStops[{originStop, stop_idx}].travelers) + L"\n";
		}

		label += L"Map Coordinates " + to_wstring(stops[stop_idx].mapCoordinates.x) + L", " + to_wstring(stops[stop_idx].mapCoordinates.y) + L"\n";
	}

	return label;
}

wstring TransportationSystem::GetLineInfo(int line_idx, TransportationStopCoordinates coords)
{
	wstring msg = L"Line Length = " + to_wstring(lines[line_idx].length) + L"\n";
	msg += L"Adjusted Line Length = " + to_wstring(lines[line_idx].fullLength) + L"\n";
	msg += L"Number Of Tunnels = " + to_wstring(lines[line_idx].numTunnels) + L"\n";
	msg += L"Number Of Vehicles = " + to_wstring(lines[line_idx].numVehicles) + L"\n";
	msg += L"Number Of Vehicles per Tunnel = " + to_wstring(lines[line_idx].vehiclesPerTunnel) + L"\n";
	//msg += L"Total Tunnel Length = " + to_wstring(lines[line_idx].GetTotalTunnelLength()) + L"\n";
	//msg += L"Total Number Of Vehicles = " + to_wstring(lines[line_idx].GetTotalVehicles()) + L"\n";

	float tolerance = 0.15;

	vector<int> line_stops = lines[line_idx].stops;
	if (lines[line_idx].bCircular)
		line_stops.push_back(line_stops[0]);

	for (int i = 0; i < line_stops.size() - 1; i++)
	{
		int stop_idx1 = line_stops[i];
		int stop_idx2 = line_stops[i + 1];

		TransportationStopCoordinates stop1_coord = stops[stop_idx1].mapCoordinates;
		TransportationStopCoordinates stop2_coord = stops[stop_idx2].mapCoordinates;

		
		bool bHorizontalRange = false;
		if ((coords.x > stop1_coord.x - tolerance && coords.x - tolerance < stop2_coord.x) || 
			(coords.x > stop2_coord.x - tolerance && coords.x - tolerance < stop1_coord.x))
			bHorizontalRange = true;

		bool bVerticalRange = false;
		if ((coords.y > stop1_coord.y - tolerance && coords.y - tolerance < stop2_coord.y) || 
			(coords.y > stop2_coord.y - tolerance && coords.y - tolerance < stop1_coord.y))
			bVerticalRange = true;

		if (bHorizontalRange && bVerticalRange) //found
		{
			int traffic1 = -1;
			if (TrafficMap.find({ stop_idx1,  stop_idx2 }) != TrafficMap.end())
				traffic1 = TrafficMap[{stop_idx1, stop_idx2}];

			msg += L"Between stops " + to_wstring(stop_idx1) + L" and " + to_wstring(stop_idx2);
			msg += L" traffic is " + to_wstring(traffic1) + L"\n";

			int traffic2 = -1;
			if (TrafficMap.find({ stop_idx2, stop_idx1 }) != TrafficMap.end())
				traffic2 = TrafficMap[{stop_idx2, stop_idx1}];

			msg += L"Between stops " + to_wstring(stop_idx2) + L" and " + to_wstring(stop_idx1);
			msg += L" traffic is " + to_wstring(traffic2) + L"\n";

			if (!TransportationSystemAssumptions::isTrainLine)
			{
				TransportationLineSegment segment = lines[line_idx].segments[{stop_idx1, stop_idx2}];
				msg += L"Between stops " + to_wstring(stop_idx1) + L" and " + to_wstring(stop_idx2) + L"\n";
				msg += L" Tunnels = " + to_wstring(segment.num_tunnels) + L"\n";
				msg += L" Approaches = " + to_wstring(segment.num_approach_tunnels) + L"\n";
				msg += L" Line Vehicles = " + to_wstring(segment.line_vehicles) + L"\n";
				msg += L" Additional Vehicles = " + to_wstring(segment.addl_vehicles) + L"\n";
				msg += L" Length = " + to_wstring(segment.length) + L"\n";
				msg += L" Total Length = " + to_wstring(segment.total_length) + L"\n";
			}
			return msg;
		}
	}
	return L"";
}

std::wstring FormatWithCommas(float f)
{
	return to_wstring(f);
}

wstring TransportationSystem::GetTotals()
{
	float vehicleCost;
	float tunnelCost;
	if (TransportationSystemAssumptions::isTrainLine)
	{
		tunnelCost = TransportationSystemAssumptions::trainTunnelCostPerKm;
		vehicleCost = TransportationSystemAssumptions::trainCost;
	}
	else
	{
		tunnelCost = TransportationSystemAssumptions::vehicleTunnelCostPerKm;
		vehicleCost = TransportationSystemAssumptions::vehicleCost;
	}
	
	wstring totals = L"Total Tunnels (km) = " + FormatWithCommas(totalTunnelLength) + L"\n";
	totals += L"Total Vehicles = " + FormatWithCommas(totalVehicles) + L"\n";
	totals += L"Total Tunnel Cost = " + FormatWithCommas(totalTunnelLength * tunnelCost) + L"\n";
	totals += L"Total Vehicle Cost = " + FormatWithCommas(totalVehicles * vehicleCost) + L"\n";
	totals += L"Total System Cost = " + FormatWithCommas(totalVehicles * vehicleCost + totalTunnelLength * tunnelCost) + L"\n";

	return totals;
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
	float Tm = TransportationSystemAssumptions::maxSpeed / TransportationSystemAssumptions::GetAcceleration();
	float maxAccelerationDistance = (TransportationSystemAssumptions::GetAcceleration() * pow(Tm, 2)) / 2;

	for (auto d : DistanceBetweenStops)
	{
		float distance = d.second;
		float time = Utils::CalculateTime(TransportationSystemAssumptions::GetAcceleration(), TransportationSystemAssumptions::maxSpeed, distance * 1000, maxAccelerationDistance);
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
		adjacency_matrix[iter->first.first][iter->first.second] = TimeBetweenStops[{iter->first.first, iter->first.second}] + adjustedStopTime;

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

	//If the origin or destination stop is a part of an intersection, 
	//we do not need to transfer even if we entered or arrived at a different stop at the same intersection
	
	
	for (auto path : PathBetweenStops)
	{
		DijkstraPath dijkstraPath = path.second;

		vector<int> origins;
		//check if origin is an intersection
		if (IntersectionMap.find(dijkstraPath.origin) != IntersectionMap.end())
			origins = intersections[IntersectionMap[dijkstraPath.origin]].stops;
		else
			origins.push_back(dijkstraPath.origin);

		vector<int> destinations;
		//check if origin is an intersection
		if (IntersectionMap.find(dijkstraPath.destination) != IntersectionMap.end())
			destinations = intersections[IntersectionMap[dijkstraPath.destination]].stops;
		else
			destinations.push_back(dijkstraPath.destination);

		//do not apply this logic if origin and destination are a part of the same intersection
		if (origins.size() != 1 && destinations.size() != 1 && IntersectionMap[dijkstraPath.origin] == IntersectionMap[dijkstraPath.destination])
			continue;
		
		if (origins.size() != 1 || destinations.size() != 1) //intersections at the origin or destination
		{
			int min_time = PathBetweenStops[{origins[0], destinations[0]}].time;
			int min_origin = origins[0];
			int min_destination = destinations[0];

			for (int origin_idx : origins)
			{
				for (int destination_idx : destinations)
				{
					if (min_origin == origin_idx && min_destination == destination_idx)
						continue;

					if (PathBetweenStops[{origin_idx, destination_idx}].time < min_time)
					{
						min_time = PathBetweenStops[{origin_idx, destination_idx}].time;
						min_origin = origin_idx;
						min_destination = destination_idx;
					}
				}
			}

			for (int origin_idx : origins)
			{
				for (int destination_idx : destinations)
				{
					if (min_origin == origin_idx && min_destination == destination_idx)
						continue;

					PathBetweenStops[{origin_idx, destination_idx}].time = min_time;
					PathBetweenStops[{origin_idx, destination_idx}].stops = PathBetweenStops[{min_origin, min_destination}].stops;
				}
			}

		}
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

void TransportationSystem::CalculateTravelMap()
{
	map<int, map<int, float>> officenums;
	map<int, float> stopShares;
	for (int i = 0; i < stops.size(); i++)
	{
		for (int j = 0; j < stops.size(); j++)
		{
			if (i == j) continue;
			vector <int> howToReach= PathBetweenStops[{i, j}].stops;
			float distance = 0.0;
			int intersectionNum = 0;
			for (int x = 0; x < howToReach.size() - 1; x++)
			{

				float dist = PathBetweenStops[{howToReach[x], howToReach[x + 1]}].distance;

				if (dist == -1)
				{
					dist = Utils::CalculateDistance(stops[howToReach[x]].mapCoordinates, stops[howToReach[x + 1]].mapCoordinates);
					PathBetweenStops[{howToReach[x], howToReach[x + 1]}].distance = dist;
				}
				distance = distance + dist;
				if (stops[howToReach[x]].line_id != stops[howToReach[x + 1]].line_id)
				{
					intersectionNum++;
				}

			}

			if (i == 5 && j == 6)
				cout << "Hello";
			
			PathBetweenStops[{i, j}].distance = distance;
			PathBetweenStops[{i, j}].intersections = intersectionNum;
			officenums[i][j] = stops[j].WorkSpaces / distance 
				* max((1.0-intersectionNum / 10.0), 0.0); //penalty for using intersections
			stopShares[i] += officenums[i][j];
		}
	}
	for (int i = 0; i < stops.size(); i++)
	{
		for (int j = 0; j < stops.size(); j++)
		{
			PathBetweenStops[{i, j}].travelers = round(stops[i].ResidentSpaces * (officenums[i][j] / stopShares[i]));
		}
	}
	int i = PathBetweenStops.size();
}

void TransportationSystem::CalculateTrafficData()
{
	for (int i = 0; i < stops.size(); i++)
	{
		for (int j = 0; j < stops.size(); j++)
		{
			if (i == j) continue;
			vector <int> pathstops = PathBetweenStops[{stops[i].id, stops[j].id}].stops;
			for (int x = 0; x < pathstops.size()-1; x++)
			{
				if (TrafficMap.find({ pathstops[x], pathstops[x+1] }) == TrafficMap.end())
					TrafficMap[{pathstops[x], pathstops[x + 1]}] = PathBetweenStops[{stops[i].id, stops[j].id}].travelers;
				else
					TrafficMap[{pathstops[x], pathstops[x + 1]}] += PathBetweenStops[{stops[i].id, stops[j].id}].travelers;
			}
		}
	}
}

void TransportationSystem::CalculateLineData()
{

	for (int i = 0; i < lines.size(); i++)
	{
		float distance = 0.0;
		int max = 0.0;
		for (int j = 0; j < lines[i].stops.size()-1; j++)
		{
			TransportationStop curr_stop = stops[lines[i].stops[j]];
			TransportationStop next_stop = stops[lines[i].stops[j+1]];
			max = j + 1;
			float distancia = PathBetweenStops[{curr_stop.id, next_stop.id}].distance;
			distance += distancia;

			
		}
		
		TransportationStop first_stop = stops[lines[i].stops[0]];
		TransportationStop last_stop = stops[lines[i].stops[max]];
		if (lines[i].bCircular) 
			distance += PathBetweenStops[{first_stop.id, last_stop.id}].distance;
		
		lines[i].length = distance;
		if (TransportationSystemAssumptions::isTrainLine)
		{
			lines[i].fullLength = distance;
		}
		else
		{
			float t = TransportationSystemAssumptions::maxSpeed / TransportationSystemAssumptions::GetAcceleration();
			float d = (TransportationSystemAssumptions::GetAcceleration() * pow(t, 2)) / 2 * 1.5;
			distance += d * 2 * lines[i].stops.size()/1000.0;
			lines[i].fullLength = distance;
		}
	}
}
void TransportationSystem::CalculateTunnelInfo()
{
	if (TransportationSystemAssumptions::isTrainLine)
	{
		adjustedStopTime = TransportationSystemAssumptions::stoptime;
	}
	else //if it's a car tunnel, the cars always travel at full speed in the main tunnel, so they need to decelerate and accelerate in off ramps, which needs to be added to stop time
	{
		float approachTime = (TransportationSystemAssumptions::maxSpeed / TransportationSystemAssumptions::GetAcceleration())*1.5;
		adjustedStopTime = round(TransportationSystemAssumptions::stoptime + approachTime * 2);
	}

	tunnelCapacity = round((3600.0 / TransportationSystemAssumptions::GetDistanceBetweenVehicles()) 
		* TransportationSystemAssumptions::GetVehicleCapacity()); //3600 because thats the number of seconds in an hour

	//this piece of the function is calculating number of vehicles needed to cover one tunnel for each line\

	if (TransportationSystemAssumptions::isTrainLine)
	{
		
		for (int i = 0; i < lines.size(); i++)
		{
			int lineTime = 0; 
			int last_idx = 0;
			int first_idx = stops[lines[i].stops[0]].id;
			for (int j = 0; j < lines[i].stops.size() -1; j++)
			{
				int curr_idx = stops[lines[i].stops[j]].id;
				int next_idx = stops[lines[i].stops[j+1]].id;
				last_idx = next_idx;
				lineTime += PathBetweenStops[{curr_idx, last_idx}].time + TransportationSystemAssumptions::stoptime;
			}
			if (lines[i].bCircular)
			{
				lineTime += PathBetweenStops[{last_idx, first_idx}].time + TransportationSystemAssumptions::stoptime;
			}
			lines[i].vehiclesPerTunnel = ceil(lineTime / TransportationSystemAssumptions::GetDistanceBetweenVehicles());
		}
		TransportationSystemAssumptions::GetDistanceBetweenVehicles();
	}
	else
	{
		float stationCapacity = TransportationSystemAssumptions::stoptime / TransportationSystemAssumptions::distanceBetweenCars;
		for (int i = 0; i < lines.size(); i++)
		{
			float dist = TransportationSystemAssumptions::GetDistanceBetweenVehicles() * TransportationSystemAssumptions::maxSpeed;
			int capacity = round((lines[i].length * 1000) / dist);
			lines[i].vehiclesPerTunnel = capacity;
			lines[i].vehiclesPerTunnel += stationCapacity*lines[i].stops.size();
		}
		//stoptime/how fast they come in = max amount of vehicles per station
	}
}
void TransportationSystem::CalculateTunnelNum()
{
	//for each line, find maximum throughput
	

	if (TransportationSystemAssumptions::isTrainLine)
	{
		for (int i = 0; i < lines.size(); i++)
		{
			int maxTraffic = 0;
			for (int j = 0; j < lines[i].stops.size() - 1; j++)
			{
				int curr_idx1 = stops[lines[i].stops[j]].id;
				int curr_idx2 = stops[lines[i].stops[j + 1]].id;
				if (TrafficMap[{curr_idx1, curr_idx2}] > maxTraffic)
				{
					maxTraffic = TrafficMap[{curr_idx1, curr_idx2}];
				}
				if (TrafficMap[{curr_idx2, curr_idx1}] > maxTraffic)
				{
					maxTraffic = TrafficMap[{curr_idx2, curr_idx1}];
				}
			}
			double maxLoad = maxTraffic / TransportationSystemAssumptions::rushHourLength;
			lines[i].numTunnels = ceil(maxLoad / (double)tunnelCapacity);
			lines[i].numVehicles = ceil((double)lines[i].vehiclesPerTunnel * maxLoad / (double)tunnelCapacity);
		}
	}
	else
	{
		
		for (int i = 0; i < lines.size(); i++)
		{
			for (int j = 0; j < lines[i].stops.size() - 1; j++)
			{
				int curr_idx = stops[lines[i].stops[j]].id;
				int next_idx = stops[lines[i].stops[j + 1]].id;
				lines[i].segments[{curr_idx, next_idx}].length = Utils::CalculateDistance(stops[curr_idx].mapCoordinates, stops[next_idx].mapCoordinates);
				int maxTraffic = TrafficMap[{curr_idx, next_idx}];
				if (TrafficMap[{curr_idx, next_idx}] < TrafficMap[{next_idx, curr_idx}])
					maxTraffic = TrafficMap[{next_idx, curr_idx}];
				double maxLoad = maxTraffic / TransportationSystemAssumptions::rushHourLength;
				float distance = TransportationSystemAssumptions::GetDistanceBetweenVehicles() * TransportationSystemAssumptions::maxSpeed;
				lines[i].segments[{curr_idx, next_idx}].num_tunnels = ceil(maxLoad / (double)tunnelCapacity);
				lines[i].segments[{curr_idx, next_idx}].line_vehicles = lines[i].segments[{curr_idx, next_idx}].length / (distance / 1000) * (maxLoad / (double)tunnelCapacity);
				lines[i].segments[{curr_idx, next_idx}].num_approach_tunnels = ceil(maxLoad / (double)tunnelCapacity) * 2;
				float d = (TransportationSystemAssumptions::GetAcceleration() * pow(TransportationSystemAssumptions::maxSpeed / TransportationSystemAssumptions::GetAcceleration(), 2)) / 2000 * 1.5;
				lines[i].segments[{curr_idx, next_idx}].total_length = (ceil(maxLoad / (double)tunnelCapacity) * 2 * d) + Utils::CalculateDistance(stops[curr_idx].mapCoordinates, stops[next_idx].mapCoordinates);
				float stationCapacity = TransportationSystemAssumptions::stoptime / TransportationSystemAssumptions::distanceBetweenCars;
				
				if (i == 0)
					lines[i].segments[{curr_idx, next_idx}].addl_vehicles = stationCapacity * lines[i].segments[{curr_idx, next_idx}].num_tunnels;
				if (i > 0 && i < lines[i].stops.size() - 2)
				{
					int maxTunnels = lines[i].segments[{curr_idx, next_idx}].num_tunnels;
					if (maxTunnels < lines[i].segments[{next_idx, next_idx + 1}].num_tunnels)
						maxTunnels = lines[i].segments[{next_idx, next_idx + 1}].num_tunnels;
					if (maxTunnels < lines[i].segments[{curr_idx - 1, curr_idx}].num_tunnels)
						maxTunnels = lines[i].segments[{curr_idx - 1, curr_idx}].num_tunnels;
					lines[i].segments[{curr_idx, next_idx}].addl_vehicles = stationCapacity * maxTunnels;
				}
				if (lines[i].bCircular)
				{
					if (i == lines[i].stops.size() - 2)
					{
						int maxTunnels = lines[i].segments[{curr_idx, next_idx}].num_tunnels;
						if (maxTunnels < lines[i].segments[{curr_idx - 1, curr_idx}].num_tunnels)
							maxTunnels = lines[i].segments[{curr_idx - 1, curr_idx}].num_tunnels;
						lines[i].segments[{curr_idx, next_idx}].addl_vehicles = stationCapacity * maxTunnels;
					}
				}
				else
				{
					if (i == lines[i].stops.size() - 2)
					{
						int maxTunnels = lines[i].segments[{curr_idx, next_idx}].num_tunnels;
						if (maxTunnels < lines[i].segments[{curr_idx - 1, curr_idx}].num_tunnels)
							maxTunnels = lines[i].segments[{curr_idx - 1, curr_idx}].num_tunnels;
						lines[i].segments[{curr_idx, next_idx}].addl_vehicles = stationCapacity * (maxTunnels + 1);
					}
				}
				
			}
		}
	}
	
}

//calculate totals for tunnel length and vehicled for the entire system
void TransportationSystem::CalculateSystemTotals()
{
	totalTunnelLength = 0;
	totalVehicles = 0;

	if (TransportationSystemAssumptions::isTrainLine)
	{
		for (int i = 0; i < lines.size(); i++)
		{
			totalTunnelLength += lines[i].fullLength * lines[i].numTunnels * 2.0; //times 2 because you want to account for traffic going in both directions
			totalVehicles += lines[i].numVehicles * 2; //times 2 because you want to account for traffic going in both directions
		}
	}
	else
	{
		for (int i = 0; i < lines.size(); i++)
		{
			for (int j = 0; j < lines[i].segments.size() - 1; j++)
			{
				totalTunnelLength += lines[i].segments[{j, j + 1}].total_length*2; //times 2 because you want to account for traffic going in both directions
				totalVehicles += (lines[i].segments[{j, j + 1}].line_vehicles+ lines[i].segments[{j, j + 1}].addl_vehicles)*2; //times 2 because you want to account for traffic going in both directions
			}
		}
	}
	
}

void TransportationSystem::CalculateSuportingInfo()
{
	CalculateStopDistances();
	CalculateStopTimes();
	CalculateIntersections();
	CalculatePaths();
	CalculatePopulationDistribution();
	CalculateTravelMap();
	CalculateTrafficData();
	CalculateLineData();
	CalculateTunnelInfo();
	CalculateTunnelNum();
	CalculateSystemTotals();
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