#pragma once
class TransportationLineSegment
{
public:
	int origin_stop_idx;
	int destination_stop_idx;

	int num_tunnels;
	int num_approach_tunnels;
	int addl_vehicles;
	int line_vehicles;

	float length;
	float total_length;

	TransportationLineSegment();
};

