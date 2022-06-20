#pragma once
#ifndef VEHICLE_MODEL_TYPEDEF
#define VEHICLE_MODEL_TYPEDEF

typedef struct vehicleParameters {
    double c1_d  = 40000;
    double c2_d  = 24000;
    double m_d   = 1920;
    double jz_d  = 2700;
    double l1_d  = 1.1615;
    double l2_d  = 1.5385;
    double swr_d = 1;
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
    double beta_d                 = 0; // rad
    double yawRate_d              = 0; // rad/s
    double yawAngle_d             = 0; // rad
    double lateralAcceleration_d  = 0; //m/s^2
    double positionX_d            = 0; // m
    double positionY_d            = 0; // m
    double longitudinalVelocity_d = 0; //m/s^2
    double lateralVelocity_d      = 0; //m/s^2
} sModelStates;

#endif
