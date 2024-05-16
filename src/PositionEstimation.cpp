#include "PositionEstimation.h"

#include <sys/time.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include "tf2/transform_datatypes.h"

cPositionEstimation::cPositionEstimation() {

}

cPositionEstimation::cPositionEstimation(bool pDynamicTimeCalcEnabled_b, int pLoopRateHz_i32, sVehicleParameters &pVehicleParameters_s, float pKinematicModelMaxSpeed_f) {
    initEstimation( pDynamicTimeCalcEnabled_b, pLoopRateHz_i32, pVehicleParameters_s, pKinematicModelMaxSpeed_f );
}

cPositionEstimation::~cPositionEstimation() {

}

void cPositionEstimation::initEstimation(bool pDynamicTimeCalcEnabled_b, int pLoopRateHz_i32, sVehicleParameters &pVehicleParameters_s, float pKinematicModelMaxSpeed_f ) {
    iFirstIteration_b = true;
    iLoopRateHz_i32 = pLoopRateHz_i32;
    iTs_d = 1 / iLoopRateHz_i32;
    iDynamicTimeCalcEnabled_b = pDynamicTimeCalcEnabled_b;
    iPrevMillisecondsSinceEpoch_u64 = 0;
    iMillisecondsSinceEpoch_u64 = 0;

    iPrevMeasPosX_d = 0;
    iPrevMeasPosY_d = 0;
    iPrevEstPosX_d = 0;
    iPrevEstPosY_d = 0;

    iPrevOrientationIsValid_b = false;

    iTravDistanceOdom_d = 0;
    iTravDistanceEstPos_d = 0;

    sVehicleParameters lVehicleParameters_s;
    lVehicleParameters_s.c1_d   = 3000;//4000;
    lVehicleParameters_s.c2_d   = 800;//2400; // The ratio is very important!!!!!
    lVehicleParameters_s.m_d    = 180;
    lVehicleParameters_s.jz_d   = 270;
    lVehicleParameters_s.l1_d   = 1.3 - 0.976;
    lVehicleParameters_s.l2_d   = 0.976;
    lVehicleParameters_s.swr_d  = 1;

    iCombinedVehicleModel_cl = cCombinedVehicleModel(lVehicleParameters_s);
    iCombinedVehicleModel_cl.initVehicleParameters(pVehicleParameters_s);
    iCombinedVehicleModel_cl.initEKFMatrices();

    iKinSpeedLimit_d = pKinematicModelMaxSpeed_f;
    iDefaultKinSpeedLimit_d = pKinematicModelMaxSpeed_f;

    iEstimationMode_e = eEstimationMode::ekf;
    iGNSSState_e = eGNSSState::off;
    iAccuracyScaleFactor_d = 10;
}

void cPositionEstimation::setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d) {
    iCombinedVehicleModel_cl.setMeasuredValuesVehicleState(pSteeringAngle_d, pVehicleSpeed_d);
}

void cPositionEstimation::setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d){
    iCombinedVehicleModel_cl.setMeasuredValuesGNSS(pPositionX_d, pPositionY_d, pPositionZ_d, pYawAngle_d);
}

void cPositionEstimation::setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d) {
    iCombinedVehicleModel_cl.setMeasuredValuesIMU(pLongitudinalAcceleration_d, pLateralAcceleration_d, pVerticalAcceleration_d, pRollRate_d, pPitchRate_d, pYawRate_d);
}

void cPositionEstimation::selectEstimationMode(int pEstimationMethod_i32, bool pGNSSStatusMsgArrived_b, int8_t pGNSSState_i8){
    iKinSpeedLimit_d = iDefaultKinSpeedLimit_d;
    iAccuracyScaleFactor_d = 10;
    iGNSSState_e = eGNSSState::off;
    iEstimationMode_e = eEstimationMode::model;

    switch (pEstimationMethod_i32) {
        case 0: // Kinematic model without EKF and without GNSS
                iKinSpeedLimit_d = 200;
            break;
        case 1: // Kinematic + dynamic model without EKF and without GNSS
            break;
        case 2: // Kinematic model without EKF and without GNSS but with yaw rate calculation on startup (based on GNSS)
                if (!iOrientationEstimation_cl.iOrientationIsValid_b) {
                    iKinSpeedLimit_d = 200;
                } else {
                    if (!iPrevOrientationIsValid_b) {
                        iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
                    }
                    iKinSpeedLimit_d = 200;
                }
                iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
            break;
        case 3: // Kinematic + dynamic model without EKF and without GNSS but with yaw rate calculation on startup (based on GNSS) 
                if (iOrientationEstimation_cl.iOrientationIsValid_b) {
                    if (!iPrevOrientationIsValid_b) {
                        iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
                    }
                }
                iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
            break;
        case 5: // Kinematic model with EKF and without GNSS
                iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                iKinSpeedLimit_d = 200;
            break;
        case 6: // Kinematic + dynaicmodel with EKF and without GNSS
                iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
            break;
        case 7: // Kinematic model with EKF and without GNSS but with yaw rate calculation on startup (based on GNSS)
                if (!iOrientationEstimation_cl.iOrientationIsValid_b) {
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                    iKinSpeedLimit_d = 200;
                } else {
                    if (!iPrevOrientationIsValid_b) {
                        iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
                    }
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                    iKinSpeedLimit_d = 200;
                }
                iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
            break;
        case 8: // Kinematic + dynamic model with EKF and without GNSS but with yaw rate calculation on startup (based on GNSS)
                if (!iOrientationEstimation_cl.iOrientationIsValid_b) {
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                } else {
                    if (!iPrevOrientationIsValid_b) {
                        iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
                    }
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                }
                iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
            break;
        case 9: // Kinematic + dynamic model with EKF and without GNSS position but with yaw rate calculation (based on GNSS)
                if (!iOrientationEstimation_cl.iOrientationIsValid_b) {
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                    iGNSSState_e = eGNSSState::SBAS;
                } else {
                    if (!iPrevOrientationIsValid_b) {
                        iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
                    }
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                    iGNSSState_e = eGNSSState::SBAS;
                }
                iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
            break;
        case 10: 
                if ((!pGNSSStatusMsgArrived_b)){
                    iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                    if (iOrientationEstimation_cl.iOrientationIsValid_b) {
                        if (!iPrevOrientationIsValid_b) {
                            iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
                        }
                    }
                    iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
                    break;
                } else {
                    iKinSpeedLimit_d = iDefaultKinSpeedLimit_d;
                    if (pGNSSState_i8 < 2) {
                        iEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                    } else {
                        iEstimationMode_e = eEstimationMode::ekf;
                        iGNSSState_e = eGNSSState::rtk_fixed;
                        iAccuracyScaleFactor_d = 1;
                    } 
                }
            break;
        default:
            break;
    }
}

void cPositionEstimation::cycleTimeCalculation(void) {
    struct timeval lTimeval_tv;
    gettimeofday(&lTimeval_tv, NULL);

    iMillisecondsSinceEpoch_u64 = 
        (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
        (unsigned long long)(lTimeval_tv.tv_usec) / 1000;

    if (iDynamicTimeCalcEnabled_b) {
        if (iPrevMillisecondsSinceEpoch_u64 == 0) {
            iTs_d = 1.0/iLoopRateHz_i32;
        } else {
            iTs_d = (double(iMillisecondsSinceEpoch_u64 - iPrevMillisecondsSinceEpoch_u64)) / 1000.0;
        }
    } else {
        iTs_d = 1.0/iLoopRateHz_i32;
    }
    iPrevMillisecondsSinceEpoch_u64 = iMillisecondsSinceEpoch_u64;
}

void cPositionEstimation::traveledDistanceCalculation(void) {
    sModelStates lPrevModelStates_st;
    sModelStates lCurrentModelStates_st;
    iCombinedVehicleModel_cl.getModelStates(&lCurrentModelStates_st);
    iCombinedVehicleModel_cl.getPrevModelStates(&lPrevModelStates_st);

    double lXDiff_d = lCurrentModelStates_st.positionX_d - lPrevModelStates_st.positionX_d;
    double lYDiff_d = lCurrentModelStates_st.positionY_d - lPrevModelStates_st.positionY_d;

    if ((abs(lXDiff_d) < 200) && (abs(lYDiff_d) < 200)) {
        iTravDistanceEstPos_d = iTravDistanceEstPos_d + sqrtf64((lXDiff_d * lXDiff_d) + (lYDiff_d * lYDiff_d));
    }

    iTravDistanceOdom_d = iTravDistanceOdom_d + iTs_d * abs(iCombinedVehicleModel_cl.iMeasuredValues_s.vehicleSpeed_d);   
}
    
void cPositionEstimation::iterateEstimation(int pEstimationMethod_i32, bool pGNSSStatusMsgArrived_b, int8_t pGNSSState_i8, bool pReset_b){
    if (iFirstIteration_b || pReset_b) {
        iCombinedVehicleModel_cl.initEKFMatrices();
        iCombinedVehicleModel_cl.setPrevEKFMatrices();
        iCombinedVehicleModel_cl.setPrevMeasuredValues();

        switch (pEstimationMethod_i32) {
            case 2:
            case 3:
            case 7:
            case 8:
            case 10:
                    iCombinedVehicleModel_cl.setModelStates(0, 
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.yawRate_d,  
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.yawAngle_d, 
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.lateralAcceleration_d,
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.positionX_d,
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.positionY_d,
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.vehicleSpeed_d,
                                                0);
                break;
            case 0:
            case 1:
            case 4:
            case 5:
            case 6:
            case 9:
            default:
                    iCombinedVehicleModel_cl.setModelStates(0, 
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.yawRate_d,  
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.yawAngle_d, 
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.lateralAcceleration_d,
                                                0,
                                                0,
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.vehicleSpeed_d,
                                                0);
                break;

        };

        iFirstIteration_b = false;

        iPrevMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.positionX_d;
        iPrevMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.positionY_d;
        iPrevEstPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.positionX_d;
        iPrevEstPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.positionY_d;
    }

    selectEstimationMode(pEstimationMethod_i32, pGNSSStatusMsgArrived_b, pGNSSState_i8);
       
    cycleTimeCalculation();

    traveledDistanceCalculation();

    iCombinedVehicleModel_cl.iterateModel(iTs_d, iEstimationMode_e, iGNSSState_e, iKinSpeedLimit_d);
    
    sModelStates lCurrentModelStates_st;
    iCombinedVehicleModel_cl.getModelStates(&lCurrentModelStates_st);

    if ( (iCombinedVehicleModel_cl.iMeasuredValues_s.vehicleSpeed_d > 0.1) &&
            (iPrevMeasPosX_d != iCombinedVehicleModel_cl.iMeasuredValues_s.positionX_d) &&
            (iPrevMeasPosY_d != iCombinedVehicleModel_cl.iMeasuredValues_s.positionY_d) &&
            (iPrevEstPosX_d != lCurrentModelStates_st.positionX_d) &&
            (iPrevEstPosY_d != lCurrentModelStates_st.positionY_d)) {

        iOrientationEstimation_cl.addPosition(  iCombinedVehicleModel_cl.iMeasuredValues_s.positionX_d, 
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.positionY_d, 
                                                lCurrentModelStates_st.positionX_d, 
                                                lCurrentModelStates_st.positionY_d);
    }

    iPrevMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.positionX_d;
    iPrevMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.positionY_d;
    iPrevEstPosX_d = lCurrentModelStates_st.positionX_d;
    iPrevEstPosY_d = lCurrentModelStates_st.positionY_d;        
}

void cPositionEstimation::getModelStates(sModelStates* pOutModelStates_s) {
    iCombinedVehicleModel_cl.getModelStates(pOutModelStates_s);
}

double cPositionEstimation::getCogDistanceFromBaselinkX(void) {
    return iCombinedVehicleModel_cl.iVehicleParameters_s.l2_d;
}

double cPositionEstimation::getCogDistanceFromBaselinkY(void) {
    return 0;
}

double cPositionEstimation::getCogDistanceFromBaselinkZ(void) {
    return 0;
}

double cPositionEstimation::getAccuracyScaleFactor(void) {
    return iAccuracyScaleFactor_d;
}

double cPositionEstimation::getFiltMeasOri(void) {
    return iOrientationEstimation_cl.iFiltMeasOri_d;
}

double cPositionEstimation::getTravDistanceOdom(void) {
    return iTravDistanceOdom_d;
}

double cPositionEstimation::getTravDistanceEstPos(void) {
    return iTravDistanceEstPos_d;
}