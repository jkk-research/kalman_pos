#pragma once
#ifndef KINEMATIC_VEHICLE_MODEL_W_EKF_W_EXT_POS1_POS2
#define KINEMATIC_VEHICLE_MODEL_W_EKF_W_EXT_POS1_POS2

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "VehicleModelTypeDef.h"

using namespace boost::numeric::ublas;

double kin_wEKFwExtPos1Pos2_MeasBasedLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_MeasBasedLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_LongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_LateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_BetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_YawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, double pLongitudinalVelocity_d, double pBeta_d);
double kin_wEKFwExtPos1Pos2_YawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_LateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d);
double kin_wEKFwExtPos1Pos2_PositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kin_wEKFwExtPos1Pos2_PositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
void   kin_wEKFwExtPos1Pos2_Estimate(sModelStates &pOutModelStates_s, matrix<double>& pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double>& pPrevP_m, matrix<double>& pQ_m, matrix<double>& pR_m);

#endif
