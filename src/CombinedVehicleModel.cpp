#include "CombinedVehicleModel.h"
#include "ros/ros.h"
#include <sstream>

cCombinedVehicleModel::cCombinedVehicleModel(std::string pVehicleType_s) {
    initEKFMatrices();
    initVehicleParameters(pVehicleType_s);
}

cCombinedVehicleModel::~cCombinedVehicleModel() {

}

void cCombinedVehicleModel::initEKFMatrices(void) {
    iPDynEKF_m = matrix<double>(5, 5);
    iQDynEKF_m = matrix<double>(5, 5);
    iRDynEKF_m = matrix<double>(5, 5);
    iPKinEKF_m = matrix<double>(5, 5);
    iQKinEKF_m = matrix<double>(5, 5);
    iRKinEKF_m = matrix<double>(5, 5);
    iPDynEKFwoGNSS_m = matrix<double>(5, 5);
    iQDynEKFwoGNSS_m = matrix<double>(5, 5);
    iRDynEKFwoGNSS_m = matrix<double>(3, 3);
    iPKinEKFwoGNSS_m = matrix<double>(5, 5);
    iQKinEKFwoGNSS_m = matrix<double>(5, 5);
    iRKinEKFwoGNSS_m = matrix<double>(3, 3);

    iPrevPDynEKF_m = matrix<double>(5, 5);
    iPrevPKinEKF_m = matrix<double>(5, 5);
    iPrevPDynEKFwoGNSS_m = matrix<double>(5, 5);
    iPrevPKinEKFwoGNSS_m = matrix<double>(5, 5);

    iPDynEKF_m(0, 0) = 1;
    iPDynEKF_m(0, 1) = 0;
    iPDynEKF_m(0, 2) = 0;
    iPDynEKF_m(0, 3) = 0;
    iPDynEKF_m(0, 4) = 0;
    iPDynEKF_m(1, 0) = 0;
    iPDynEKF_m(1, 1) = 1;
    iPDynEKF_m(1, 2) = 0;
    iPDynEKF_m(1, 3) = 0;
    iPDynEKF_m(1, 4) = 0;
    iPDynEKF_m(2, 0) = 0;
    iPDynEKF_m(2, 1) = 0;
    iPDynEKF_m(2, 2) = 1;
    iPDynEKF_m(2, 3) = 0;
    iPDynEKF_m(2, 4) = 0;
    iPDynEKF_m(3, 0) = 0;
    iPDynEKF_m(3, 1) = 0;
    iPDynEKF_m(3, 2) = 0;
    iPDynEKF_m(3, 3) = 1;
    iPDynEKF_m(3, 4) = 0;
    iPDynEKF_m(4, 0) = 0;
    iPDynEKF_m(4, 1) = 0;
    iPDynEKF_m(4, 2) = 0;
    iPDynEKF_m(4, 3) = 0;
    iPDynEKF_m(4, 4) = 1;

    iQDynEKF_m(0, 0) = 0.0145;
    iQDynEKF_m(0, 1) = 0;
    iQDynEKF_m(0, 2) = 0;
    iQDynEKF_m(0, 3) = 0;
    iQDynEKF_m(0, 4) = 0;
    iQDynEKF_m(1, 0) = 0;
    iQDynEKF_m(1, 1) = 1.4821;
    iQDynEKF_m(1, 2) = 0;
    iQDynEKF_m(1, 3) = 0;
    iQDynEKF_m(1, 4) = 0;
    iQDynEKF_m(2, 0) = 0;
    iQDynEKF_m(2, 1) = 0;
    iQDynEKF_m(2, 2) = 1.929482792522855e-04;
    iQDynEKF_m(2, 3) = 0;
    iQDynEKF_m(2, 4) = 0;
    iQDynEKF_m(3, 0) = 0;
    iQDynEKF_m(3, 1) = 0;
    iQDynEKF_m(3, 2) = 0;
    iQDynEKF_m(3, 3) = 2.002;
    iQDynEKF_m(3, 4) = 0;
    iQDynEKF_m(4, 0) = 0;
    iQDynEKF_m(4, 1) = 0;
    iQDynEKF_m(4, 2) = 0;
    iQDynEKF_m(4, 3) = 0;
    iQDynEKF_m(4, 4) = 1.9060;

    iRDynEKF_m(0, 0) = 3.729484658649690e-04;
    iRDynEKF_m(0, 1) = 0;
    iRDynEKF_m(0, 2) = 0;
    iRDynEKF_m(0, 3) = 0;
    iRDynEKF_m(0, 4) = 0;
    iRDynEKF_m(1, 0) = 0;
    iRDynEKF_m(1, 1) = 1.750794749765446;
    iRDynEKF_m(1, 2) = 0;
    iRDynEKF_m(1, 3) = 0;
    iRDynEKF_m(1, 4) = 0;
    iRDynEKF_m(2, 0) = 0;
    iRDynEKF_m(2, 1) = 0;
    iRDynEKF_m(2, 2) = 1.543640292721019;
    iRDynEKF_m(2, 3) = 0;
    iRDynEKF_m(2, 4) = 0;
    iRDynEKF_m(3, 0) = 0;
    iRDynEKF_m(3, 1) = 0;
    iRDynEKF_m(3, 2) = 0;
    iRDynEKF_m(3, 3) = 10;
    iRDynEKF_m(3, 4) = 0;
    iRDynEKF_m(4, 0) = 0;
    iRDynEKF_m(4, 1) = 0;
    iRDynEKF_m(4, 2) = 0;
    iRDynEKF_m(4, 3) = 0;
    iRDynEKF_m(4, 4) = 10;

    iPKinEKF_m(0, 0) = 1;
    iPKinEKF_m(0, 1) = 0;
    iPKinEKF_m(0, 2) = 0;
    iPKinEKF_m(0, 3) = 0;
    iPKinEKF_m(0, 4) = 0;
    iPKinEKF_m(1, 0) = 0;
    iPKinEKF_m(1, 1) = 1;
    iPKinEKF_m(1, 2) = 0;
    iPKinEKF_m(1, 3) = 0;
    iPKinEKF_m(1, 4) = 0;
    iPKinEKF_m(2, 0) = 0;
    iPKinEKF_m(2, 1) = 0;
    iPKinEKF_m(2, 2) = 1;
    iPKinEKF_m(2, 3) = 0;
    iPKinEKF_m(2, 4) = 0;
    iPKinEKF_m(3, 0) = 0;
    iPKinEKF_m(3, 1) = 0;
    iPKinEKF_m(3, 2) = 0;
    iPKinEKF_m(3, 3) = 1;
    iPKinEKF_m(3, 4) = 0;
    iPKinEKF_m(4, 0) = 0;
    iPKinEKF_m(4, 1) = 0;
    iPKinEKF_m(4, 2) = 0;
    iPKinEKF_m(4, 3) = 0;
    iPKinEKF_m(4, 4) = 1;

    iQKinEKF_m(0, 0) = 1.750800000000000;
    iQKinEKF_m(0, 1) = 0;
    iQKinEKF_m(0, 2) = 0;
    iQKinEKF_m(0, 3) = 0;
    iQKinEKF_m(0, 4) = 0;
    iQKinEKF_m(1, 0) = 0;
    iQKinEKF_m(1, 1) = 1.750800000000000;
    iQKinEKF_m(1, 2) = 0;
    iQKinEKF_m(1, 3) = 0;
    iQKinEKF_m(1, 4) = 0;
    iQKinEKF_m(2, 0) = 0;
    iQKinEKF_m(2, 1) = 0;
    iQKinEKF_m(2, 2) = 2;
    iQKinEKF_m(2, 3) = 0;
    iQKinEKF_m(2, 4) = 0;
    iQKinEKF_m(3, 0) = 0;
    iQKinEKF_m(3, 1) = 0;
    iQKinEKF_m(3, 2) = 0;
    iQKinEKF_m(3, 3) = 2;
    iQKinEKF_m(3, 4) = 0;
    iQKinEKF_m(4, 0) = 0;
    iQKinEKF_m(4, 1) = 0;
    iQKinEKF_m(4, 2) = 0;
    iQKinEKF_m(4, 3) = 0;
    iQKinEKF_m(4, 4) = 0.02;

    iRKinEKF_m(0, 0) = 0.5;
    iRKinEKF_m(0, 1) = 0;
    iRKinEKF_m(0, 2) = 0;
    iRKinEKF_m(0, 3) = 0;
    iRKinEKF_m(0, 4) = 0;
    iRKinEKF_m(1, 0) = 0;
    iRKinEKF_m(1, 1) = 0.5;
    iRKinEKF_m(1, 2) = 0;
    iRKinEKF_m(1, 3) = 0;
    iRKinEKF_m(1, 4) = 0;
    iRKinEKF_m(2, 0) = 0;
    iRKinEKF_m(2, 1) = 0;
    iRKinEKF_m(2, 2) = 10;
    iRKinEKF_m(2, 3) = 0;
    iRKinEKF_m(2, 4) = 0;
    iRKinEKF_m(3, 0) = 0;
    iRKinEKF_m(3, 1) = 0;
    iRKinEKF_m(3, 2) = 0;
    iRKinEKF_m(3, 3) = 10;
    iRKinEKF_m(3, 4) = 0;
    iRKinEKF_m(4, 0) = 0;
    iRKinEKF_m(4, 1) = 0;
    iRKinEKF_m(4, 2) = 0;
    iRKinEKF_m(4, 3) = 0;
    iRKinEKF_m(4, 4) = 1.75;

    iPDynEKFwoGNSS_m(0, 0) = 1;
    iPDynEKFwoGNSS_m(0, 1) = 0;
    iPDynEKFwoGNSS_m(0, 2) = 0;
    iPDynEKFwoGNSS_m(0, 3) = 0;
    iPDynEKFwoGNSS_m(0, 4) = 0;
    iPDynEKFwoGNSS_m(1, 0) = 0;
    iPDynEKFwoGNSS_m(1, 1) = 1;
    iPDynEKFwoGNSS_m(1, 2) = 0;
    iPDynEKFwoGNSS_m(1, 3) = 0;
    iPDynEKFwoGNSS_m(1, 4) = 0;
    iPDynEKFwoGNSS_m(2, 0) = 0;
    iPDynEKFwoGNSS_m(2, 1) = 0;
    iPDynEKFwoGNSS_m(2, 2) = 1;
    iPDynEKFwoGNSS_m(2, 3) = 0;
    iPDynEKFwoGNSS_m(2, 4) = 0;
    iPDynEKFwoGNSS_m(3, 0) = 0;
    iPDynEKFwoGNSS_m(3, 1) = 0;
    iPDynEKFwoGNSS_m(3, 2) = 0;
    iPDynEKFwoGNSS_m(3, 3) = 1;
    iPDynEKFwoGNSS_m(3, 4) = 0;
    iPDynEKFwoGNSS_m(4, 0) = 0;
    iPDynEKFwoGNSS_m(4, 1) = 0;
    iPDynEKFwoGNSS_m(4, 2) = 0;
    iPDynEKFwoGNSS_m(4, 3) = 0;
    iPDynEKFwoGNSS_m(4, 4) = 1;

    iQDynEKFwoGNSS_m(0, 0) = 0.0145;
    iQDynEKFwoGNSS_m(0, 1) = 0;
    iQDynEKFwoGNSS_m(0, 2) = 0;
    iQDynEKFwoGNSS_m(0, 3) = 0;
    iQDynEKFwoGNSS_m(0, 4) = 0;
    iQDynEKFwoGNSS_m(1, 0) = 0;
    iQDynEKFwoGNSS_m(1, 1) = 1.4821;
    iQDynEKFwoGNSS_m(1, 2) = 0;
    iQDynEKFwoGNSS_m(1, 3) = 0;
    iQDynEKFwoGNSS_m(1, 4) = 0;
    iQDynEKFwoGNSS_m(2, 0) = 0;
    iQDynEKFwoGNSS_m(2, 1) = 0;
    iQDynEKFwoGNSS_m(2, 2) = 1.929482792522855e-04;
    iQDynEKFwoGNSS_m(2, 3) = 0;
    iQDynEKFwoGNSS_m(2, 4) = 0;
    iQDynEKFwoGNSS_m(3, 0) = 0;
    iQDynEKFwoGNSS_m(3, 1) = 0;
    iQDynEKFwoGNSS_m(3, 2) = 0;
    iQDynEKFwoGNSS_m(3, 3) = 2.002;
    iQDynEKFwoGNSS_m(3, 4) = 0;
    iQDynEKFwoGNSS_m(4, 0) = 0;
    iQDynEKFwoGNSS_m(4, 1) = 0;
    iQDynEKFwoGNSS_m(4, 2) = 0;
    iQDynEKFwoGNSS_m(4, 3) = 0;
    iQDynEKFwoGNSS_m(4, 4) = 1.9060;

    iRDynEKFwoGNSS_m(0, 0) = 3.729484658649690e-04;
    iRDynEKFwoGNSS_m(0, 1) = 0;
    iRDynEKFwoGNSS_m(0, 2) = 0;
    iRDynEKFwoGNSS_m(1, 0) = 0;
    iRDynEKFwoGNSS_m(1, 1) = 1.750794749765446; //1.750794749765446;
    iRDynEKFwoGNSS_m(1, 2) = 0;
    iRDynEKFwoGNSS_m(2, 0) = 0;
    iRDynEKFwoGNSS_m(2, 1) = 0;
    iRDynEKFwoGNSS_m(2, 2) = 1.543640292721019;

    iPKinEKFwoGNSS_m(0, 0) = 1;
    iPKinEKFwoGNSS_m(0, 1) = 0;
    iPKinEKFwoGNSS_m(0, 2) = 0;
    iPKinEKFwoGNSS_m(0, 3) = 0;
    iPKinEKFwoGNSS_m(0, 4) = 0;
    iPKinEKFwoGNSS_m(1, 0) = 0;
    iPKinEKFwoGNSS_m(1, 1) = 1;
    iPKinEKFwoGNSS_m(1, 2) = 0;
    iPKinEKFwoGNSS_m(1, 3) = 0;
    iPKinEKFwoGNSS_m(1, 4) = 0;
    iPKinEKFwoGNSS_m(2, 0) = 0;
    iPKinEKFwoGNSS_m(2, 1) = 0;
    iPKinEKFwoGNSS_m(2, 2) = 1;
    iPKinEKFwoGNSS_m(2, 3) = 0;
    iPKinEKFwoGNSS_m(2, 4) = 0;
    iPKinEKFwoGNSS_m(3, 0) = 0;
    iPKinEKFwoGNSS_m(3, 1) = 0;
    iPKinEKFwoGNSS_m(3, 2) = 0;
    iPKinEKFwoGNSS_m(3, 3) = 1;
    iPKinEKFwoGNSS_m(3, 4) = 0;
    iPKinEKFwoGNSS_m(4, 0) = 0;
    iPKinEKFwoGNSS_m(4, 1) = 0;
    iPKinEKFwoGNSS_m(4, 2) = 0;
    iPKinEKFwoGNSS_m(4, 3) = 0;
    iPKinEKFwoGNSS_m(4, 4) = 1;

    iQKinEKFwoGNSS_m(0, 0) = 1.750800000000000;
    iQKinEKFwoGNSS_m(0, 1) = 0;
    iQKinEKFwoGNSS_m(0, 2) = 0;
    iQKinEKFwoGNSS_m(0, 3) = 0;
    iQKinEKFwoGNSS_m(0, 4) = 0;
    iQKinEKFwoGNSS_m(1, 0) = 0;
    iQKinEKFwoGNSS_m(1, 1) = 1.750800000000000;
    iQKinEKFwoGNSS_m(1, 2) = 0;
    iQKinEKFwoGNSS_m(1, 3) = 0;
    iQKinEKFwoGNSS_m(1, 4) = 0;
    iQKinEKFwoGNSS_m(2, 0) = 0;
    iQKinEKFwoGNSS_m(2, 1) = 0;
    iQKinEKFwoGNSS_m(2, 2) = 2;
    iQKinEKFwoGNSS_m(2, 3) = 0;
    iQKinEKFwoGNSS_m(2, 4) = 0;
    iQKinEKFwoGNSS_m(3, 0) = 0;
    iQKinEKFwoGNSS_m(3, 1) = 0;
    iQKinEKFwoGNSS_m(3, 2) = 0;
    iQKinEKFwoGNSS_m(3, 3) = 2;
    iQKinEKFwoGNSS_m(3, 4) = 0;
    iQKinEKFwoGNSS_m(4, 0) = 0;
    iQKinEKFwoGNSS_m(4, 1) = 0;
    iQKinEKFwoGNSS_m(4, 2) = 0;
    iQKinEKFwoGNSS_m(4, 3) = 0;
    iQKinEKFwoGNSS_m(4, 4) = 0.02;

    iRKinEKFwoGNSS_m(0, 0) = 0.5;
    iRKinEKFwoGNSS_m(0, 1) = 0;
    iRKinEKFwoGNSS_m(0, 2) = 0;
    iRKinEKFwoGNSS_m(1, 0) = 0;
    iRKinEKFwoGNSS_m(1, 1) = 0.5;
    iRKinEKFwoGNSS_m(1, 2) = 0;
    iRKinEKFwoGNSS_m(2, 0) = 0;
    iRKinEKFwoGNSS_m(2, 1) = 0;
    iRKinEKFwoGNSS_m(2, 2) = 1.75;

    setPrevEKFMatrices();
}

void cCombinedVehicleModel::setPrevEKFMatrices(void) {
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            iPrevPDynEKF_m(i, j)        = iPDynEKF_m(i, j);
            iPrevPKinEKF_m(i, j)        = iPKinEKF_m(i, j);
            iPrevPDynEKFwoGNSS_m(i, j)  = iPDynEKFwoGNSS_m(i, j);
            iPrevPKinEKFwoGNSS_m(i, j)  = iPKinEKFwoGNSS_m(i, j);
        }
    }
}

void cCombinedVehicleModel::initVehicleParameters(std::string pVehicleType_s) {
    if (pVehicleType_s == "leaf") {
        iVehicleParameters_s.c1_d   = 40000;
        iVehicleParameters_s.c2_d   = 24000;
        iVehicleParameters_s.m_d    = 1920;
        iVehicleParameters_s.jz_d   = 2700;
        iVehicleParameters_s.l1_d   = 1.1615;
        iVehicleParameters_s.l2_d   = 1.5385;
        iVehicleParameters_s.swr_d  = 1;
    } else if (pVehicleType_s == "SZEmission") {
        iVehicleParameters_s.c1_d   = 3000;//4000;
        iVehicleParameters_s.c2_d   = 800;//2400; // The ratio is very important!!!!!
        iVehicleParameters_s.m_d    = 180;
        iVehicleParameters_s.jz_d   = 270;
        iVehicleParameters_s.l1_d   = 1.3 - 0.976;
        iVehicleParameters_s.l2_d   = 0.976;
        iVehicleParameters_s.swr_d  = 1;
    } else {
        iVehicleParameters_s.c1_d   = 40000;
        iVehicleParameters_s.c2_d   = 24000;
        iVehicleParameters_s.m_d    = 1920;
        iVehicleParameters_s.jz_d   = 2700;
        iVehicleParameters_s.l1_d   = 1.1615;
        iVehicleParameters_s.l2_d   = 1.5385;
        iVehicleParameters_s.swr_d  = 1;
    }
}

void cCombinedVehicleModel::setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d) {
    iMeasuredValues_s.steeringAngle_d   = pSteeringAngle_d / iVehicleParameters_s.swr_d;
    iMeasuredValues_s.vehicleSpeed_d    = pVehicleSpeed_d;
}

void cCombinedVehicleModel::setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d) {
    iMeasuredValues_s.positionX_d = pPositionX_d;
    iMeasuredValues_s.positionY_d = pPositionY_d;
    iMeasuredValues_s.positionZ_d = pPositionZ_d;
    iMeasuredValues_s.yawAngle_d  = pYawAngle_d;

}

void cCombinedVehicleModel::setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d) {
    iMeasuredValues_s.longitudinalAcceleration_d = pLongitudinalAcceleration_d;
    iMeasuredValues_s.lateralAcceleration_d      = pLateralAcceleration_d;
    iMeasuredValues_s.verticalAcceleration_d     = pVerticalAcceleration_d;
    iMeasuredValues_s.rollRate_d                 = pRollRate_d;
    iMeasuredValues_s.pitchRate_d                = pPitchRate_d;
    iMeasuredValues_s.yawRate_d                  = pYawRate_d;
}

void cCombinedVehicleModel::setPrevMeasuredValues() {
    iPrevMeasuredValues_s.steeringAngle_d            = iMeasuredValues_s.steeringAngle_d;
    iPrevMeasuredValues_s.vehicleSpeed_d             = iMeasuredValues_s.vehicleSpeed_d;
    iPrevMeasuredValues_s.positionX_d                = iMeasuredValues_s.positionX_d;
    iPrevMeasuredValues_s.positionY_d                = iMeasuredValues_s.positionY_d;
    iPrevMeasuredValues_s.positionZ_d                = iMeasuredValues_s.positionZ_d;
    iPrevMeasuredValues_s.yawAngle_d                 = iMeasuredValues_s.yawAngle_d;
    iPrevMeasuredValues_s.longitudinalAcceleration_d = iMeasuredValues_s.longitudinalAcceleration_d;
    iPrevMeasuredValues_s.lateralAcceleration_d      = iMeasuredValues_s.lateralAcceleration_d;
    iPrevMeasuredValues_s.verticalAcceleration_d     = iMeasuredValues_s.verticalAcceleration_d;
    iPrevMeasuredValues_s.rollRate_d                 = iMeasuredValues_s.rollRate_d;
    iPrevMeasuredValues_s.pitchRate_d                = iMeasuredValues_s.pitchRate_d;
    iPrevMeasuredValues_s.yawRate_d                  = iMeasuredValues_s.yawRate_d;
}

void cCombinedVehicleModel::setModelStates(double pBeta_d, double pYawRate_d, double pYawAngle_d, double pLateralAcceleration_d, double pPositionX_d, double pPositionY_d, double pLongitudinalVelocity_d, double pLateralVelocity_d) {
    iModelStates_s.beta_d                 = pBeta_d;
    iModelStates_s.lateralAcceleration_d  = pLateralAcceleration_d;
    iModelStates_s.positionX_d            = pPositionX_d;
    iModelStates_s.positionY_d            = pPositionY_d;
    iModelStates_s.yawAngle_d             = pYawAngle_d;
    iModelStates_s.yawRate_d              = pYawRate_d;
    iModelStates_s.lateralVelocity_d      = pLateralVelocity_d;
    iModelStates_s.longitudinalVelocity_d = pLongitudinalVelocity_d;
}

void cCombinedVehicleModel::setPrevModelStates(void) {
    iPrevModelStates_s.beta_d                 = iModelStates_s.beta_d;
    iPrevModelStates_s.lateralAcceleration_d  = iModelStates_s.lateralAcceleration_d;
    iPrevModelStates_s.positionX_d            = iModelStates_s.positionX_d;
    iPrevModelStates_s.positionY_d            = iModelStates_s.positionY_d;
    iPrevModelStates_s.yawAngle_d             = iModelStates_s.yawAngle_d;
    iPrevModelStates_s.yawRate_d              = iModelStates_s.yawRate_d;
    iPrevModelStates_s.lateralVelocity_d      = iModelStates_s.lateralVelocity_d;
    iPrevModelStates_s.longitudinalVelocity_d = iModelStates_s.longitudinalVelocity_d;

    /*ROS_INFO_STREAM("set prev state: beta" << iModelStates_s.beta_d  
        << " ay: " << iModelStates_s.lateralAcceleration_d 
        << " x: " << iModelStates_s.positionX_d 
        << " y: " << iModelStates_s.positionY_d
        << " yaw_a: " << iModelStates_s.yawAngle_d
        << " yaw_r: " << iModelStates_s.yawRate_d
        << " v_x: " << iModelStates_s.lateralVelocity_d
        << " v_y: " << iModelStates_s.longitudinalVelocity_d
    );*/

}

void cCombinedVehicleModel::iterateModel(double pTs_d, eEstimationMode pEstimationMode_e, eGNSSState pGNSSState_e, double pKinSpeedLimit_d) {
    double lBeta_d                = 0;
    double lLateralAcceleration_d = 0;
    double lPositionX_d           = 0;
    double lPositionY_d           = 0;
    double lYawAngle_d            = 0;
    double lYawRate_d             = 0;
    double lLateralSpeed_d        = 0;
    double lLongitudinalSpeed_d   = 0;
    
    if (iPrevMeasuredValues_s.vehicleSpeed_d < pKinSpeedLimit_d) {
        if ((pEstimationMode_e == eEstimationMode::model)) {
            // Kinematic model
            setPrevModelStates();
            setPrevEKFMatrices();

            
            lBeta_d                 = kinBetaCalculation(iVehicleParameters_s, iMeasuredValues_s, iModelStates_s, pTs_d);
            //lYawRate_d              = iMeasuredValues_s.yawRate_d;//kinYawRateCalculation(iVehicleParameters_s, iMeasuredValues_s, iModelStates_s, pTs_d);
            lYawRate_d              = kinYawRateCalculation(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawAngle_d             = kinYawAngleCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralAcceleration_d  = iMeasuredValues_s.lateralAcceleration_d;//kinLateralAccCalculation(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
            lPositionX_d            = kinPositionXCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lPositionY_d            = kinPositionYCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLongitudinalSpeed_d    = kinLongitudinalVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
            lLateralSpeed_d         = kinLateralVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
            
            /*
            lBeta_d                 = kinBetaCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawRate_d              = kinYawRateCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawAngle_d             = kinYawAngleCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralAcceleration_d  = kinLateralAccCalculation2(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
            lPositionX_d            = kinPositionXCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lPositionY_d            = kinPositionYCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLongitudinalSpeed_d    = kinLongitudinalVelocityCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralSpeed_d         = kinLateralVelocityCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            */

            //ROS_INFO_STREAM("Model  Beta: " << lBeta_d << "  YR: " << lYawRate_d << "  YA: " << lYawAngle_d << "  LA: " << lLateralAcceleration_d << "  X: " << lPositionX_d << "  Y: " << lPositionY_d << "  VX: " << lLongitudinalSpeed_d << "  VY: " << lLateralSpeed_d);
            setModelStates(lBeta_d, lYawRate_d, lYawAngle_d, lLateralAcceleration_d, lPositionX_d, lPositionY_d, lLongitudinalSpeed_d, lLateralSpeed_d);
            setPrevMeasuredValues();
        }
        else if ((pEstimationMode_e >= eEstimationMode::ekf) && (pGNSSState_e >= eGNSSState::rtk_float)) {
            // Kinematic model with EKF + GNSS
            setPrevModelStates();
            setPrevEKFMatrices();
            //ROS_INFO_STREAM("GNSS");
            kinEKFEstimate(
                iModelStates_s,
                iPKinEKF_m,
                iVehicleParameters_s,
                iMeasuredValues_s,
                iPrevMeasuredValues_s,
                iPrevModelStates_s, 
                pTs_d, 
                iPrevPKinEKF_m,
                iQKinEKF_m, 
                iRKinEKF_m);

            /*
            ROS_INFO_STREAM("-set prev state: beta" << iModelStates_s.beta_d  
                    << " ay: " << iModelStates_s.lateralAcceleration_d 
                    << " x: " << iModelStates_s.positionX_d 
                    << " y: " << iModelStates_s.positionY_d
                    << " yaw_a: " << iModelStates_s.yawAngle_d
                    << " yaw_r: " << iModelStates_s.yawRate_d
                    << " v_x: " << iModelStates_s.lateralVelocity_d
                    << " v_y: " << iModelStates_s.longitudinalVelocity_d);
            */

            setPrevMeasuredValues();
        }
        else {
            // Kinematic model with EKF without GNSS
            //ROS_INFO_STREAM("woGNSS");
            setPrevModelStates();
            setPrevEKFMatrices();

            kinEKFwoGNSSEstimate(
                iModelStates_s,
                iPKinEKFwoGNSS_m,
                iVehicleParameters_s,
                iMeasuredValues_s,
                iPrevMeasuredValues_s,
                iPrevModelStates_s,
                pTs_d,
                iPrevPKinEKFwoGNSS_m,
                iQKinEKFwoGNSS_m,
                iRKinEKFwoGNSS_m);

            //ROS_INFO_STREAM("Model EKF wo GNSS Beta: " << lBeta_d << "  YR: " << lYawRate_d << "  YA: " << lYawAngle_d << "  LA: " << lLateralAcceleration_d << "  X: " << lPositionX_d << "  Y: " << lPositionY_d << "  VX: " << lLongitudinalSpeed_d << "  VY: " << lLateralSpeed_d);
            setPrevMeasuredValues();
        }

    } else {
        if ((pEstimationMode_e == eEstimationMode::model)) {
            // Dynamic model
            //ROS_INFO_STREAM("Dynamic model");
            setPrevModelStates();
            setPrevEKFMatrices();

            lBeta_d                 = dynBetaCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawRate_d              = dynYawRateCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawAngle_d             = dynYawAngleCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralAcceleration_d  = dynLateralAccCalculation(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
            lPositionX_d            = dynPositionXCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lPositionY_d            = dynPositionYCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLongitudinalSpeed_d    = dynLongitudinalVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
            lLateralSpeed_d         = dynLateralVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
         

            /*
            lBeta_d                 = kinBetaCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawRate_d              = kinYawRateCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawAngle_d             = kinYawAngleCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralAcceleration_d  = kinLateralAccCalculation2(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
            lPositionX_d            = kinPositionXCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lPositionY_d            = kinPositionYCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLongitudinalSpeed_d    = kinLongitudinalVelocityCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralSpeed_d         = kinLateralVelocityCalculation2(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            */

            setModelStates(lBeta_d, lYawRate_d, lYawAngle_d, lLateralAcceleration_d, lPositionX_d, lPositionY_d, lLongitudinalSpeed_d, lLateralSpeed_d);
            setPrevMeasuredValues();

                /*
                ROS_INFO_STREAM("-set prev state: beta" << iModelStates_s.beta_d  
                    << " ay: " << iModelStates_s.lateralAcceleration_d 
                    << " x: " << iModelStates_s.positionX_d 
                    << " y: " << iModelStates_s.positionY_d
                    << " yaw_a: " << iModelStates_s.yawAngle_d
                    << " yaw_r: " << iModelStates_s.yawRate_d
                    << " v_x: " << iModelStates_s.lateralVelocity_d
                    << " v_y: " << iModelStates_s.longitudinalVelocity_d);
                */
        }
        else if ((pEstimationMode_e >= eEstimationMode::ekf) && (pGNSSState_e >= eGNSSState::rtk_float)) {
            // Dynamic model with EKF + GNSS
            //ROS_INFO_STREAM("Dynamic model EKF + GNSS");
            setPrevModelStates();
            setPrevEKFMatrices();

            dynEKFEstimate(
                iModelStates_s,
                iPDynEKF_m,
                iVehicleParameters_s,
                iMeasuredValues_s,
                iPrevMeasuredValues_s,
                iPrevModelStates_s,
                pTs_d,
                iPrevPDynEKF_m,
                iQDynEKF_m,
                iRDynEKF_m);

            setPrevMeasuredValues();
        }else {
            // Dynamic model with EKF without GNSS
            //ROS_INFO_STREAM("Dynamic model EKF wo GNSS");
            setPrevModelStates();
            setPrevEKFMatrices();

            if (pGNSSState_e > eGNSSState::off) {
                dynEKFwoGNSSEstimate(
                    iModelStates_s,
                    iPDynEKFwoGNSS_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s,
                    pTs_d,
                    iPrevPDynEKFwoGNSS_m,
                    iQDynEKFwoGNSS_m,
                    iRDynEKFwoGNSS_m,
                    true);
            } else {
                dynEKFwoGNSSEstimate(
                    iModelStates_s,
                    iPDynEKFwoGNSS_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s,
                    pTs_d,
                    iPrevPDynEKFwoGNSS_m,
                    iQDynEKFwoGNSS_m,
                    iRDynEKFwoGNSS_m,
                    false);
            }

            //ROS_INFO_STREAM("Model EKF wo GNSS Beta: " << lBeta_d << "  YR: " << lYawRate_d << "  YA: " << lYawAngle_d << "  LA: " << lLateralAcceleration_d << "  X: " << lPositionX_d << "  Y: " << lPositionY_d << "  VX: " << lLongitudinalSpeed_d << "  VY: " << lLateralSpeed_d);
            setPrevMeasuredValues();
        }
    }
}

void cCombinedVehicleModel::getModelStates(sModelStates* pOutModelStates_s) {
    pOutModelStates_s->beta_d                 = iModelStates_s.beta_d;
    pOutModelStates_s->lateralAcceleration_d  = iModelStates_s.lateralAcceleration_d;
    pOutModelStates_s->positionX_d            = iModelStates_s.positionX_d;
    pOutModelStates_s->positionY_d            = iModelStates_s.positionY_d;
    pOutModelStates_s->yawAngle_d             = iModelStates_s.yawAngle_d;
    pOutModelStates_s->yawRate_d              = iModelStates_s.yawRate_d;
    pOutModelStates_s->lateralVelocity_d      = iModelStates_s.lateralVelocity_d;
    pOutModelStates_s->longitudinalVelocity_d = iModelStates_s.longitudinalVelocity_d;
}

void cCombinedVehicleModel::setYawAngleStates(double pYawAngle_d) {
    iModelStates_s.yawAngle_d = pYawAngle_d;
    iPrevModelStates_s.yawAngle_d = pYawAngle_d;
} 

 double cCombinedVehicleModel::getYawAngle(void) {
    return iModelStates_s.yawAngle_d;
 }