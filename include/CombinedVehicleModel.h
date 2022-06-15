#pragma once
#ifndef COMBINED_VEHICLE_MODEL
#define COMBINED_VEHICLE_MODEL

typedef struct vehicleParameters {
    double c1_d     = 40000;
    double c2_d     = 24000;
    double m_d      = 1920;
    double jz_d     = 2700;
    double l1_d     = 1.1615;
    double l2_d     = 1.5385;
    double swr_d    = 1;
} sVehicleParameters;
    
typedef struct measuredValues {
    double steeringAngle_d              = 0; // rad
    double vehicleSpeed_d               = 0; // m/s
    double longitudinalAcceleration_d   = 0; // m/s^2
    double lateralAcceleration_d        = 0; // m/s^2
    double verticalAcceleration_d       = 0; // m/s^2
    double rollRate_d                   = 0; // rad/s
    double pitchRate_d                  = 0; // rad/s
    double yawRate_d                    = 0; // rad/s
    double positionX_d                  = 0; // m
    double positionY_d                  = 0; // m
    double positionZ_d                  = 0; // m
    double yawAngle_d                   = 0; // rad
} sMeasuredValues;

typedef struct modelStates {
    double beta_d                = 0; // rad
    double yawRate_d             = 0; // rad/s
    double yawAngle_d            = 0; // rad
    double lateralAcceleration_d = 0; //m/s^2
    double positionX_d           = 0; // m
    double positionY_d           = 0; // m
} sModelStates;

class cCombinedVehicleModel {
private:
    // Private Variables
    sVehicleParameters  iVehicleParameters_s;
    sMeasuredValues     iMeasuredValues_s;
    sMeasuredValues     iPrevMeasuredValues_s;
    sModelStates        iModelStates_s;
    sModelStates        iPrevModelStates_s;

public:
    // Public functions
    cCombinedVehicleModel();
    ~cCombinedVehicleModel();
    void initVehicleParameters(void);
    void setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d);
    void setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d);
    void setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d);
    void setPrevMeasuredValues(void);
    void setModelStates(double pBeta_d, double pYawRate_d, double pYawAngle_d, double pLateralAcceleration_d, double pPositionX_d, double pPositionY_d);
    void setPrevModelStates(void);
    void iterateModel(double pTs_d);
    void getModelStates(sModelStates* pOutModelStates_s);
};
#endif

