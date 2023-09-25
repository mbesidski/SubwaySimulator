#include "TransportationSystemAssumptions.h"

float TransportationSystemAssumptions::proximity_distance = 0.05; //km
float TransportationSystemAssumptions::trainAcceleration = 1; // m/s^2
float TransportationSystemAssumptions::carAcceleration = 3; // m/s^2
float TransportationSystemAssumptions::maxSpeed = 30; // m/s

float TransportationSystemAssumptions::OuterCityDiameter = 30; //km
float TransportationSystemAssumptions::MapSize = OuterCityDiameter + 2;
float TransportationSystemAssumptions::InnerCityDiameter = 6;
float TransportationSystemAssumptions::MiddleCityDiameter = 15;
float TransportationSystemAssumptions::RadialLineStep = 45.0;
float TransportationSystemAssumptions::OuterCityStopLength = 1.5;
float TransportationSystemAssumptions::MiddleCityStopLength = 1.5;
float TransportationSystemAssumptions::InnerCityStopLength = 1;
int TransportationSystemAssumptions::transferTime = 180; //sec - transfer to another line on intersection
int TransportationSystemAssumptions::addlTransferTime = 300; //sec - penalty for additinal pair of lines past two on intersection transfer
int TransportationSystemAssumptions::stoptime = 45; //sec
int TransportationSystemAssumptions::distanceBetweenTrains = 110; //sec
int TransportationSystemAssumptions::distanceBetweenCars = 3; //sec
bool TransportationSystemAssumptions::isTrainLine = false;
bool TransportationSystemAssumptions::isHeatMap = false;
int TransportationSystemAssumptions::carCapacity = 15; //people
int TransportationSystemAssumptions::trainCapacity = 2200; //people
int TransportationSystemAssumptions::rushHourLength = 3; //hours
int TransportationSystemAssumptions::CommutingPopulation = 1000000*6;

int TransportationSystemAssumptions::vehicleTunnelCostPerKm = 10000000; //based on boring company's own projections for the Las Vegas Loop... 
int TransportationSystemAssumptions::vehicleCost = 100000; //dollars

int TransportationSystemAssumptions::trainTunnelCostPerKm = 33000000; //Chongqing metro/3 because we are doing the cost per track, and we have an average of three tracks at any given point
int TransportationSystemAssumptions::trainCost = 20000000; //dollars

TransportationStopCoordinates TransportationSystemAssumptions::MapCenter()
{
	return TransportationStopCoordinates(MapSize / 2, MapSize / 2);
}

float TransportationSystemAssumptions::GetAcceleration()
{
	if (isTrainLine)
	{
		return trainAcceleration;
	}
	else
	{
		return carAcceleration;
	}
}
float TransportationSystemAssumptions::GetDistanceBetweenVehicles()
{
	if (isTrainLine)
	{
		return distanceBetweenTrains;
	}
	else
	{
		return distanceBetweenCars;
	}
}
int TransportationSystemAssumptions::GetVehicleCapacity()
{
	if (isTrainLine)
	{
		return trainCapacity;
	}
	else
	{
		return carCapacity;
	}
}