#pragma once
#ifndef DYNAMIC_VEHICLE_MODEL
#define DYNAMIC_VEHICLE_MODEL

#include "CombinedVehicleModel.h"

double dynBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d);
double dynPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);

#endif