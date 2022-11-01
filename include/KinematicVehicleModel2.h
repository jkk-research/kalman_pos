#pragma once
#ifndef KINEMATIC_VEHICLE_MODEL
#define KINEMATIC_VEHICLE_MODEL

#include "VehicleModelTypeDef.h"

double kinLongitudinalVelocityCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinLateralVelocityCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinBetaCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinYawRateCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinYawAngleCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinLateralAccCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d);
double kinPositionXCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinPositionYCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);

#endif
