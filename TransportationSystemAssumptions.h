#pragma once
class TransportationSystemAssumptions
{
public:
	static float acceleration; // m/s^2
	static float maxSpeed; // m/s
	static float proximity_distance; // km - minimal distance between stops to not count them as an intersection

	static float OuterCityDiameter;
	static float MapSize;
	static float InnerCityDiameter;
	static float MiddleCityDiameter;
	static float RadialLineStep;
	static float OuterCityStopLength;
	static float InnerCityStopLength;
	static int transferTime;
	static int addlTransferTime;
};

