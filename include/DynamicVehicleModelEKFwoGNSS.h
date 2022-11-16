#pragma once
#ifndef DYNAMIC_VEHICLE_MODEL_EKF_WO_GNSS
#define DYNAMIC_VEHICLE_MODEL_EKF_WO_GNSS

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "VehicleModelTypeDef.h"

using namespace boost::numeric::ublas;

double dynEKFwoGNSSLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d);
double dynEKFwoGNSSLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d);
double dynEKFwoGNSSBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynEKFwoGNSSYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynEKFwoGNSSYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynEKFwoGNSSLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d);
double dynEKFwoGNSSPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double dynEKFwoGNSSPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
void   dynEKFwoGNSSEstimate(sModelStates &pOutModelStates_s, matrix<double>& pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double>& pPrevP_m, matrix<double>& pQ_m, matrix<double>& pR_m,  bool pUseYawAngle_b);

#endif
