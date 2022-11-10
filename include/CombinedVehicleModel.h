#pragma once
#ifndef COMBINED_VEHICLE_MODEL
#define COMBINED_VEHICLE_MODEL

#include "VehicleModelTypeDef.h"

#include "DynamicVehicleModel.h"
#include "DynamicVehicleModelEKF.h"
#include "DynamicVehicleModelEKFwoGNSS.h"
#include "KinematicVehicleModel.h"
#include "KinematicVehicleModel2.h"
#include "KinematicVehicleModelEKF.h"
#include "KinematicVehicleModelEKFwoGNSS.h"

#include <sstream>

using namespace boost::numeric::ublas;

enum class eEstimationMode { model = 0,  model_ekf = 1, ekf_ekf_wognss = 2, ekf = 3};
enum class eGNSSState {off = 0, SBAS = 1, pseudorange = 2, rtk_float = 3, rtk_fixed = 4};

class cCombinedVehicleModel {
private:
    // Private Variables
    sVehicleParameters  iVehicleParameters_s;
    sMeasuredValues     iMeasuredValues_s;
    sMeasuredValues     iPrevMeasuredValues_s;
    sModelStates        iModelStates_s;
    sModelStates        iPrevModelStates_s;
    matrix<double>      iPDynEKF_m;
    matrix<double>      iQDynEKF_m;
    matrix<double>      iRDynEKF_m;
    matrix<double>      iPKinEKF_m;
    matrix<double>      iQKinEKF_m;
    matrix<double>      iRKinEKF_m;
    matrix<double>      iPDynEKFwoGNSS_m;
    matrix<double>      iQDynEKFwoGNSS_m;
    matrix<double>      iRDynEKFwoGNSS_m;
    matrix<double>      iPKinEKFwoGNSS_m;
    matrix<double>      iQKinEKFwoGNSS_m;
    matrix<double>      iRKinEKFwoGNSS_m;
    matrix<double>      iPrevPDynEKF_m;
    matrix<double>      iPrevPKinEKF_m;
    matrix<double>      iPrevPDynEKFwoGNSS_m;
    matrix<double>      iPrevPKinEKFwoGNSS_m;

public:
    // Public functions
    cCombinedVehicleModel(std::string pVehicleType_s);
    ~cCombinedVehicleModel();
    void initEKFMatrices(void);
    void setPrevEKFMatrices(void);
    void initVehicleParameters(std::string pVehicleType_s);
    void setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d);
    void setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d);
    void setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d);
    void setPrevMeasuredValues(void);
    void setModelStates(double pBeta_d, double pYawRate_d, double pYawAngle_d, double pLateralAcceleration_d, double pPositionX_d, double pPositionY_d, double pLongitudinalVelocity_d, double pLateralVelocity_d);
    void setPrevModelStates(void);
    void iterateModel(double pTs_d, eEstimationMode pEstimationMode_e, eGNSSState pGNSSState);
    void getModelStates(sModelStates* pOutModelStates_s);
    void setYawAngleStates(double pYawAngle_d);
    double getYawAngle(void);
};
#endif
