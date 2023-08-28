#pragma once
#include "TransportationStopCoordinates.h"

class TransportationSystemAssumptions
{
public:
	static float trainAcceleration; // m/s^2
	static float carAcceleration; // m/s^2
	static float maxSpeed; // m/s
	static float proximity_distance; // km - minimal distance between stops to not count them as an intersection

	static float OuterCityDiameter; //km
	static float MapSize; //km
	static float InnerCityDiameter; //km
	static float MiddleCityDiameter; //km
	static float RadialLineStep; //degrees
	static float OuterCityStopLength; //km
	static float MiddleCityStopLength; //km
	static float InnerCityStopLength; //km
	static int transferTime; //sec
	static int addlTransferTime; //sec
	static int stoptime; //sec
	static int distanceBetweenTrains; //sec
	static int distanceBetweenCars; //sec
	static int carCapacity; //max people in one car
	static int trainCapacity; //max people in one train
	static int rushHourLength; //hours

	static bool isTrainLine; //train if true, car if not

	static int CommutingPopulation; //we assume one commuter per household for simplicity
	
	static TransportationStopCoordinates MapCenter();

	static float GetAcceleration();
	static float GetDistanceBetweenVehicles();
	static int GetVehicleCapacity();
};

