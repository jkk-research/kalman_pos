#pragma once
#ifndef KINEMATIC_VEHICLE_MODEL
#define KINEMATIC_VEHICLE_MODEL

#include "CombinedVehicleModel.h"

double kinBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d);
double kinYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d);
double kinYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d);
double kinPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);

#endif