#pragma once
#ifndef VEHICLE_MODEL_TYPEDEF
#define VEHICLE_MODEL_TYPEDEF

typedef struct vehicleParameters {
    double iC1_d  = 40000;
    double iC2_d  = 24000;
    double iM_d   = 1920;
    double iJz_d  = 2700;
    double iL1_d  = 1.1615;
    double iL2_d  = 1.5385;
    double iSwr_d = 1;
} sVehicleParameters;

typedef struct measuredValues {
    double iSteeringAngle_d              = 0; // rad
    double iVehicleSpeed_d               = 0; // m/s
    double iLongitudinalAcceleration_d   = 0; // m/s^2
    double iLateralAcceleration_d        = 0; // m/s^2
    double iVerticalAcceleration_d       = 0; // m/s^2
    double iRollRate_d                   = 0; // rad/s
    double iPitchRate_d                  = 0; // rad/s
    double iYawRate_d                    = 0; // rad/s
    double iPosition1X_d                 = 0; // m
    double iPosition1Y_d                 = 0; // m
    double iPosition1Z_d                 = 0; // m
    double iPosition2X_d                 = 0; // m
    double iPosition2Y_d                 = 0; // m
    double iPosition2Z_d                 = 0; // m
    double iYawAngle1_d                  = 0; // rad
    double iYawAngle2_d                  = 0; // rad
} sMeasuredValues;

typedef struct modelStates {
    double iBeta_d                 = 0; // rad
    double iYawRate_d              = 0; // rad/s
    double iYawAngle_d             = 0; // rad
    double iLateralAcceleration_d  = 0; //m/s^2
    double iPositionX_d            = 0; // m
    double iPositionY_d            = 0; // m
    double iLongitudinalVelocity_d = 0; //m/s^2
    double iLateralVelocity_d      = 0; //m/s^2
} sModelStates;

#endif
