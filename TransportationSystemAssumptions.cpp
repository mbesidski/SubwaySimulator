#include "TransportationSystemAssumptions.h"

float TransportationSystemAssumptions::proximity_distance = 0.05; //km
float TransportationSystemAssumptions::acceleration = 1; // m/s^2
float TransportationSystemAssumptions::maxSpeed = 30; // m/s

float TransportationSystemAssumptions::OuterCityDiameter = 30; //km
float TransportationSystemAssumptions::MapSize = OuterCityDiameter + 2;
float TransportationSystemAssumptions::InnerCityDiameter = 6;
float TransportationSystemAssumptions::MiddleCityDiameter = 15;
float TransportationSystemAssumptions::RadialLineStep = 45.0;
float TransportationSystemAssumptions::OuterCityStopLength = 4;
float TransportationSystemAssumptions::InnerCityStopLength = OuterCityStopLength / 4.0;
int TransportationSystemAssumptions::transferTime = 120; //sec - transfer to another line on intersection
int TransportationSystemAssumptions::addlTransferTime = 300; //sec - penalty for additinal pair of lines past two on intersection transfer

int TransportationSystemAssumptions::CommutingPopulation = 2000000;
