#include "TransportationSystemAssumptions.h"

float TransportationSystemAssumptions::proximity_distance = 0.05;
float TransportationSystemAssumptions::acceleration = 1;
float TransportationSystemAssumptions::maxSpeed = 30;

float TransportationSystemAssumptions::OuterCityDiameter = 30;
float TransportationSystemAssumptions::MapSize = OuterCityDiameter + 2;
float TransportationSystemAssumptions::InnerCityDiameter = 6;
float TransportationSystemAssumptions::MiddleCityDiameter = 15;
float TransportationSystemAssumptions::RadialLineStep = 45.0;
float TransportationSystemAssumptions::OuterCityStopLength = 4;
float TransportationSystemAssumptions::InnerCityStopLength = OuterCityStopLength / 4.0;
int TransportationSystemAssumptions::transferTime = 60; //sec - transfer to another line on intersection
int TransportationSystemAssumptions::addlTransferTime = 60; //sec - penalty for additinal line past two on intersection transfer
