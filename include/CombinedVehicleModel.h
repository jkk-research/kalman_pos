#pragma once
#ifndef COMBINED_VEHICLE_MODEL
#define COMBINED_VEHICLE_MODEL

#include "VehicleModelTypeDef.h"

#include "DynamicVehicleModel_woEKFwoExtPos.h"
#include "DynamicVehicleModel_wEKFwExtPos1.h"
#include "DynamicVehicleModel_wEKFwExtPos2.h"
#include "DynamicVehicleModel_wEKFwExtPos1Pos2.h"
#include "DynamicVehicleModel_wEKFwoExtPos.h"
#include "KinematicVehicleModel_woEKFwoExtPos.h"
#include "KinematicVehicleModel_wEKFwExtPos1.h"
#include "KinematicVehicleModel_wEKFwExtPos2.h"
#include "KinematicVehicleModel_wEKFwExtPos1Pos2.h"
#include "KinematicVehicleModel_wEKFwoExtPos.h"

#include <sstream>

using namespace boost::numeric::ublas;

class cCombinedVehicleModel {
private:
    // Private Variables
    sMeasuredValues     iPrevMeasuredValues_s;
    sModelStates        iModelStates_s;
    sModelStates        iPrevModelStates_s;
    matrix<double>      iPDyn_wEKFwExtPos1_m;
    matrix<double>      iQDyn_wEKFwExtPos1_m;
    matrix<double>      iRDyn_wEKFwExtPos1_m;
    matrix<double>      iPKin_wEKFwExtPos1_m;
    matrix<double>      iQKin_wEKFwExtPos1_m;
    matrix<double>      iRKin_wEKFwExtPos1_m;
    matrix<double>      iPDyn_wEKFwExtPos2_m;
    matrix<double>      iQDyn_wEKFwExtPos2_m;
    matrix<double>      iRDyn_wEKFwExtPos2_m;
    matrix<double>      iPKin_wEKFwExtPos2_m;
    matrix<double>      iQKin_wEKFwExtPos2_m;
    matrix<double>      iRKin_wEKFwExtPos2_m;
    matrix<double>      iPDyn_wEKFwExtPos1Pos2_m;
    matrix<double>      iQDyn_wEKFwExtPos1Pos2_m;
    matrix<double>      iRDyn_wEKFwExtPos1Pos2_m;
    matrix<double>      iPKin_wEKFwExtPos1Pos2_m;
    matrix<double>      iQKin_wEKFwExtPos1Pos2_m;
    matrix<double>      iRKin_wEKFwExtPos1Pos2_m;
    matrix<double>      iPDyn_wEKFwoExtPos_m;
    matrix<double>      iQDyn_wEKFwoExtPos_m;
    matrix<double>      iRDyn_wEKFwoExtPos_m;
    matrix<double>      iPKin_wEKFwoExtPos_m;
    matrix<double>      iQKin_wEKFwoExtPos_m;
    matrix<double>      iRKin_wEKFwoExtPos_m;
    matrix<double>      iPrevPDyn_wEKFwExtPos1_m;
    matrix<double>      iPrevPKin_wEKFwExtPos1_m;
    matrix<double>      iPrevPDyn_wEKFwExtPos2_m;
    matrix<double>      iPrevPKin_wEKFwExtPos2_m;
    matrix<double>      iPrevPDyn_wEKFwExtPos1Pos2_m;
    matrix<double>      iPrevPKin_wEKFwExtPos1Pos2_m;
    matrix<double>      iPrevPDyn_wEKFwoExtPos_m;
    matrix<double>      iPrevPKin_wEKFwoExtPos_m;

public:
    // Public Variables
    sVehicleParameters  iVehicleParameters_s;
    sMeasuredValues     iMeasuredValues_s;

public:
    // Public functions
    cCombinedVehicleModel();
    cCombinedVehicleModel( sVehicleParameters &pVehicleParameters_s);
    ~cCombinedVehicleModel();
    void initEKFMatrices(void);
    void setPrevEKFMatrices(void);
    void initVehicleParameters(  sVehicleParameters &pVehicleParameters_s);
    void setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d);
    void setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d);
    void setMeasuredValuesSLAM(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d);
    void setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d);
    void setPrevMeasuredValues(void);
    void setModelStates(double pBeta_d, double pYawRate_d, double pYawAngle_d, double pLateralAcceleration_d, double pPositionX_d, double pPositionY_d, double pLongitudinalVelocity_d, double pLateralVelocity_d);
    void getPrevModelStates(sModelStates* pOutPrevModelStates_s);
    void setPrevModelStates(void);
    void iterateModel(double pTs_d, bool pUseRawModel_b, bool pGNSSAvailable_b, bool pSLAMAvailable_b, double pKinSpeedLimit_d);
    void getModelStates(sModelStates* pOutModelStates_s);
    void setYawAngleStates(double pYawAngle_d);
    double getYawAngle(void);
    void setPositionCovariance(double pCovariancePos1X_d, double pCovariancePos1Y_d, double pCovariancePos2X_d, double pCovariancePos2Y_d);
    void setRKin_wEKFwExtPos1(matrix<double> pR_m);
    void setRDyn_wEKFwExtPos1(matrix<double> pR_m);
    void setRKin_wEKFwExtPos2(matrix<double> pR_m);
    void setRDyn_wEKFwExtPos2(matrix<double> pR_m);
    void setRKin_wEKFwExtPos1Pos2(matrix<double> pR_m);
    void setRDyn_wEKFwExtPos1Pos2(matrix<double> pR_m);
    void setRKin_wEKFwoExtPos(matrix<double> pR_m);
    void setRDyn_wEKFwoExtPos(matrix<double> pR_m);
};
#endif
