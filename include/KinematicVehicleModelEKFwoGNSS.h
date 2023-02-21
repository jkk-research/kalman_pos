#pragma once
#ifndef KINEMATIC_VEHICLE_MODEL_EKF_WO_GNSS
#define KINEMATIC_VEHICLE_MODEL_EKF_WO_GNSS

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "VehicleModelTypeDef.h"

using namespace boost::numeric::ublas;

double kinEKFwoGNSSMeasBasedLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d);
double kinEKFwoGNSSMeasBasedLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d);
double kinEKFwoGNSSLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinEKFwoGNSSLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinEKFwoGNSSBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d);
double kinEKFwoGNSSYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, double pLongitudinalVelocity_d, double pBeta_d);
double kinEKFwoGNSSYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinEKFwoGNSSLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d);
double kinEKFwoGNSSPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
double kinEKFwoGNSSPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d);
void   kinEKFwoGNSSEstimate(sModelStates &pOutModelStates_s, matrix<double>& pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double>& pPrevP_m, matrix<double>& pQ_m, matrix<double>& pR_m);

#endif
