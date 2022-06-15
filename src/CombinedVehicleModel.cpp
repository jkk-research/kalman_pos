#include "CombinedVehicleModel.h"
#include "DynamicVehicleModel.h"
#include "KinematicVehicleModel.h"

cCombinedVehicleModel::cCombinedVehicleModel() {
    initVehicleParameters();
}

cCombinedVehicleModel::~cCombinedVehicleModel() {

}

void cCombinedVehicleModel::initVehicleParameters(void) {
    iVehicleParameters_s.c1_d   = 40000;
    iVehicleParameters_s.c2_d   = 24000;
    iVehicleParameters_s.m_d    = 1920;
    iVehicleParameters_s.jz_d   = 2700;
    iVehicleParameters_s.l1_d   = 1.1615;
    iVehicleParameters_s.l2_d   = 1.5385;
    iVehicleParameters_s.swr_d  = 1;
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

void cCombinedVehicleModel::setModelStates(double pBeta_d, double pYawRate_d, double pYawAngle_d, double pLateralAcceleration_d, double pPositionX_d, double pPositionY_d) {
    iModelStates_s.beta_d                = pBeta_d;
    iModelStates_s.lateralAcceleration_d = pLateralAcceleration_d;
    iModelStates_s.positionX_d           = pPositionX_d;
    iModelStates_s.positionY_d           = pPositionY_d;
    iModelStates_s.yawAngle_d            = pYawAngle_d;
    iModelStates_s.yawRate_d             = pYawRate_d;
}

void cCombinedVehicleModel::setPrevModelStates(void) {
    iPrevModelStates_s.beta_d                = iModelStates_s.beta_d;
    iPrevModelStates_s.lateralAcceleration_d = iModelStates_s.lateralAcceleration_d;
    iPrevModelStates_s.positionX_d           = iModelStates_s.positionX_d;
    iPrevModelStates_s.positionY_d           = iModelStates_s.positionY_d;
    iPrevModelStates_s.yawAngle_d            = iModelStates_s.yawAngle_d;
    iPrevModelStates_s.yawRate_d             = iModelStates_s.yawRate_d;
}

void cCombinedVehicleModel::iterateModel(double pTs_d) {
    double lBeta_d                = 0;
    double lLateralAcceleration_d = 0;
    double lPositionX_d           = 0;
    double lPositionY_d           = 0;
    double lYawAngle_d            = 0;
    double lYawRate_d             = 0;
    
    if (iPrevMeasuredValues_s.vehicleSpeed_d < 5) {
        // Kinematic model

        lBeta_d                 = kinBetaCalculation(iVehicleParameters_s, iMeasuredValues_s, iModelStates_s, pTs_d);
        lYawRate_d              = kinYawRateCalculation(iVehicleParameters_s, iMeasuredValues_s, iModelStates_s, pTs_d);
        lYawAngle_d             = kinYawAngleCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
        lLateralAcceleration_d  = kinLateralAccCalculation(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
        lPositionX_d            = kinPositionXCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
        lPositionY_d            = kinPositionYCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);

        setPrevModelStates();
        setModelStates(lBeta_d, lYawRate_d, lYawAngle_d, lLateralAcceleration_d, lPositionX_d, lPositionY_d);
    }
    else {
        // Dynamic model

        lBeta_d                = dynBetaCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
        lYawRate_d             = dynYawRateCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
        lYawAngle_d            = dynYawAngleCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
        lLateralAcceleration_d = dynLateralAccCalculation(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
        lPositionX_d           = dynPositionXCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
        lPositionY_d           = dynPositionYCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
    
        setPrevModelStates();
        setModelStates(lBeta_d, lYawRate_d, lYawAngle_d, lLateralAcceleration_d, lPositionX_d, lPositionY_d);
    }
}

void cCombinedVehicleModel::getModelStates(sModelStates* pOutModelStates_s) {
    pOutModelStates_s->beta_d                = iModelStates_s.beta_d;
    pOutModelStates_s->lateralAcceleration_d = iModelStates_s.lateralAcceleration_d;
    pOutModelStates_s->positionX_d           = iModelStates_s.positionX_d;
    pOutModelStates_s->positionY_d           = iModelStates_s.positionY_d;
    pOutModelStates_s->yawAngle_d            = iModelStates_s.yawAngle_d;
    pOutModelStates_s->yawRate_d             = iModelStates_s.yawRate_d;

}