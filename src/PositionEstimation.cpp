#include "PositionEstimation.h"

#include <sys/time.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include "tf2/transform_datatypes.h"

cPositionEstimation::cPositionEstimation() {

}

cPositionEstimation::cPositionEstimation(bool pDynamicTimeCalcEnabled_b, int pLoopRateHz_i32, sVehicleParameters &pVehicleParameters_s, float pKinematicModelMaxSpeed_f, bool pOriEstimationEnabled_b) {
    initEstimation( pDynamicTimeCalcEnabled_b, pLoopRateHz_i32, pVehicleParameters_s, pKinematicModelMaxSpeed_f, pOriEstimationEnabled_b);
}

cPositionEstimation::~cPositionEstimation() {

}

void cPositionEstimation::initEstimation(bool pDynamicTimeCalcEnabled_b, int pLoopRateHz_i32, sVehicleParameters &pVehicleParameters_s, float pKinematicModelMaxSpeed_f, bool pOriEstimationEnabled_b ) {
    iFirstIteration_b = true;
    iLoopRateHz_i32 = pLoopRateHz_i32;
    iTs_d = 1 / iLoopRateHz_i32;
    iDynamicTimeCalcEnabled_b = pDynamicTimeCalcEnabled_b;
    iPrevMillisecondsSinceEpoch_u64 = 0;
    iMillisecondsSinceEpoch_u64 = 0;
    iOriEstimationEnabled_b = pOriEstimationEnabled_b;

    iPrevGNSSMeasPosX_d = 0;
    iAccuracyScaleFactor_d = 0;
    iPrevSLAMMeasPosX_d = 0;
    iPrevSLAMMeasPosY_d = 0;
    iPrevEstPosX_d = 0;
    iPrevEstPosY_d = 0;

    iPrevOrientationIsValid_b = false;

    iTravDistanceOdom_d = 0;
    iTravDistanceEstPos_d = 0;

    sVehicleParameters lVehicleParameters_s;
    lVehicleParameters_s.iC1_d   = 3000;//4000;
    lVehicleParameters_s.iC2_d   = 800;//2400; // The ratio is very important!!!!!
    lVehicleParameters_s.iM_d    = 180;
    lVehicleParameters_s.iJz_d   = 270;
    lVehicleParameters_s.iL1_d   = 1.3 - 0.976;
    lVehicleParameters_s.iL2_d   = 0.976;
    lVehicleParameters_s.iSwr_d  = 1;

    iCombinedVehicleModel_cl = cCombinedVehicleModel(lVehicleParameters_s);
    iCombinedVehicleModel_cl.initVehicleParameters(pVehicleParameters_s);
    iCombinedVehicleModel_cl.initEKFMatrices();

    iKinSpeedLimit_d = pKinematicModelMaxSpeed_f;
    iDefaultKinSpeedLimit_d = pKinematicModelMaxSpeed_f;

    iAccuracyScaleFactor_d = 10;
}

void cPositionEstimation::setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d) {
    iCombinedVehicleModel_cl.setMeasuredValuesVehicleState(pSteeringAngle_d, pVehicleSpeed_d);
}

void cPositionEstimation::setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d){
    iCombinedVehicleModel_cl.setMeasuredValuesGNSS(pPositionX_d, pPositionY_d, pPositionZ_d, pYawAngle_d);
}

void cPositionEstimation::setMeasuredValuesSLAM(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d){
    iCombinedVehicleModel_cl.setMeasuredValuesSLAM(pPositionX_d, pPositionY_d, pPositionZ_d, pYawAngle_d);
}

void cPositionEstimation::setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d) {
    iCombinedVehicleModel_cl.setMeasuredValuesIMU(pLongitudinalAcceleration_d, pLateralAcceleration_d, pVerticalAcceleration_d, pRollRate_d, pPitchRate_d, pYawRate_d);
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

    double lXDiff_d = lCurrentModelStates_st.iPositionX_d - lPrevModelStates_st.iPositionX_d;
    double lYDiff_d = lCurrentModelStates_st.iPositionY_d - lPrevModelStates_st.iPositionY_d;

    if ((abs(lXDiff_d) < 200) && (abs(lYDiff_d) < 200)) {
        iTravDistanceEstPos_d = iTravDistanceEstPos_d + sqrtf64((lXDiff_d * lXDiff_d) + (lYDiff_d * lYDiff_d));
    }

    iTravDistanceOdom_d = iTravDistanceOdom_d + iTs_d * abs(iCombinedVehicleModel_cl.iMeasuredValues_s.iVehicleSpeed_d);   
}
    
void cPositionEstimation::iterateEstimation(bool pUseRawModel_b, bool pGNSSAvailable_b, bool pSLAMAvailable_b, double pGNSSCovariance_da[3], double pSLAMCovariance_da[3], bool pReset_b){
    if (iFirstIteration_b || pReset_b) {
        iCombinedVehicleModel_cl.initEKFMatrices();
        iCombinedVehicleModel_cl.setPrevEKFMatrices();
        iCombinedVehicleModel_cl.setPrevMeasuredValues();
        iCombinedVehicleModel_cl.setPositionCovariance(pGNSSCovariance_da[0], pGNSSCovariance_da[1], pSLAMCovariance_da[0], pSLAMCovariance_da[1]);

        if (pGNSSAvailable_b) {
            iCombinedVehicleModel_cl.setModelStates(0, 
                iCombinedVehicleModel_cl.iMeasuredValues_s.iYawRate_d,  
                iCombinedVehicleModel_cl.iMeasuredValues_s.iYawAngle1_d, 
                iCombinedVehicleModel_cl.iMeasuredValues_s.iLateralAcceleration_d,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iVehicleSpeed_d,
                0);
            iPrevGNSSMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d;
            iPrevGNSSMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d;
            iPrevSLAMMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d;
            iPrevSLAMMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d;
            iPrevEstPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d;
            iPrevEstPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d;
        } else if (pSLAMAvailable_b) {
            iCombinedVehicleModel_cl.setModelStates(0, 
                iCombinedVehicleModel_cl.iMeasuredValues_s.iYawRate_d,  
                iCombinedVehicleModel_cl.iMeasuredValues_s.iYawAngle1_d, 
                iCombinedVehicleModel_cl.iMeasuredValues_s.iLateralAcceleration_d,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iVehicleSpeed_d,
                0);
            iPrevGNSSMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d;
            iPrevGNSSMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d;
            iPrevSLAMMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d;
            iPrevSLAMMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d;
            iPrevEstPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d;
            iPrevEstPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d;
        } else {
            iCombinedVehicleModel_cl.setModelStates(0, 
                iCombinedVehicleModel_cl.iMeasuredValues_s.iYawRate_d,  
                iCombinedVehicleModel_cl.iMeasuredValues_s.iYawAngle1_d, 
                iCombinedVehicleModel_cl.iMeasuredValues_s.iLateralAcceleration_d,
                0,
                0,
                iCombinedVehicleModel_cl.iMeasuredValues_s.iVehicleSpeed_d,
                0);
            iPrevGNSSMeasPosX_d = 0;
            iPrevGNSSMeasPosY_d = 0;
            iPrevSLAMMeasPosX_d = 0;
            iPrevSLAMMeasPosY_d = 0;
            iPrevEstPosX_d = 0;
            iPrevEstPosY_d = 0;
        }

        iFirstIteration_b = false;
    }

    if (iOriEstimationEnabled_b) {
        if (iOrientationEstimation_cl.iOrientationIsValid_b) {
            if (!iPrevOrientationIsValid_b) {
                iCombinedVehicleModel_cl.setYawAngleStates(iOrientationEstimation_cl.iFiltMeasOri_d);
            }
        }
        iPrevOrientationIsValid_b = iOrientationEstimation_cl.iOrientationIsValid_b;
    }

    cycleTimeCalculation();

    traveledDistanceCalculation();

    iCombinedVehicleModel_cl.iterateModel(iTs_d, pUseRawModel_b, pGNSSAvailable_b, pSLAMAvailable_b, iKinSpeedLimit_d);
    
    sModelStates lCurrentModelStates_st;
    iCombinedVehicleModel_cl.getModelStates(&lCurrentModelStates_st);

    if (pGNSSAvailable_b) {
        if ( (iCombinedVehicleModel_cl.iMeasuredValues_s.iVehicleSpeed_d > 0.1) &&
                (iPrevGNSSMeasPosX_d != iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d) &&
                (iPrevGNSSMeasPosY_d != iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d) &&
                (iPrevEstPosX_d != lCurrentModelStates_st.iPositionX_d) &&
                (iPrevEstPosY_d != lCurrentModelStates_st.iPositionY_d)) {

            iOrientationEstimation_cl.addPosition(  iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d, 
                                                    iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d, 
                                                    lCurrentModelStates_st.iPositionX_d, 
                                                    lCurrentModelStates_st.iPositionY_d);
        }
    } else if (pSLAMAvailable_b) {
        if ( (iCombinedVehicleModel_cl.iMeasuredValues_s.iVehicleSpeed_d > 0.1) &&
            (iPrevSLAMMeasPosX_d != iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d) &&
            (iPrevSLAMMeasPosY_d != iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d) &&
            (iPrevEstPosX_d != lCurrentModelStates_st.iPositionX_d) &&
            (iPrevEstPosY_d != lCurrentModelStates_st.iPositionY_d)) {

        iOrientationEstimation_cl.addPosition(  iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d, 
                                                iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d, 
                                                lCurrentModelStates_st.iPositionX_d, 
                                                lCurrentModelStates_st.iPositionY_d);
        }
    }

    iPrevGNSSMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1X_d;
    iPrevGNSSMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition1Y_d;
    iPrevSLAMMeasPosX_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2X_d;
    iPrevSLAMMeasPosY_d = iCombinedVehicleModel_cl.iMeasuredValues_s.iPosition2Y_d;
    iPrevEstPosX_d = lCurrentModelStates_st.iPositionX_d;
    iPrevEstPosY_d = lCurrentModelStates_st.iPositionY_d;        
}

void cPositionEstimation::getModelStates(sModelStates* pOutModelStates_s) {
    iCombinedVehicleModel_cl.getModelStates(pOutModelStates_s);
}

double cPositionEstimation::getCogDistanceFromBaselinkX(void) {
    return iCombinedVehicleModel_cl.iVehicleParameters_s.iL2_d;
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