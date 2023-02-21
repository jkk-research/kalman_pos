#include "OrientationEstimation.h"

#include <math.h>
#include <cstring>
#include "ros/ros.h"

cOrientationEstimation::cOrientationEstimation() {
    init();
}

cOrientationEstimation::~cOrientationEstimation() {

}

void cOrientationEstimation::init() {
    for (int i = 0; i < POS_BUFFER_SIZE; i++) {
        iMeasPosBufferX_da[i] = 0;
        iMeasPosBufferY_da[i] = 0;
        iEstPosBufferX_da[i] = 0;
        iEstPosBufferY_da[i] = 0;
    }

    for (int i = 0; i < ORIENTATION_DIF_BUFFER_SIZE; i++) {
        iMeasOriBuffer_da[i] = INVALID_ORIENTATION;
        iEstOriBuffer_da[i] = INVALID_ORIENTATION;
        iMeasEstOriBufferDif_da[i] = INVALID_ORIENTATION;
        iFiltMeasEstOriBufferDif_da[i] = INVALID_ORIENTATION;
    }

    iFilteredMeanDifference_d = INVALID_ORIENTATION;
    iFiltMeasOri_d = INVALID_ORIENTATION;
    iFiltEstOri_d = INVALID_ORIENTATION;
    iMeasEstOriBufferDifValid_b= false;
    iPosBufferIndex_s32 = 0;
    iMeasEstOriBufferDifIndex_s32 = 0;
    iOrientationIsValid_b = false;
}

void cOrientationEstimation::addPosition(double pMeasPosX_d, double pMeasPosY_d, double pEstPosX_d, double pEstPosY_d){
    if (iPosBufferIndex_s32 < POS_BUFFER_SIZE) {
        iMeasPosBufferX_da[iPosBufferIndex_s32] = pMeasPosX_d;
        iMeasPosBufferY_da[iPosBufferIndex_s32] = pMeasPosY_d;
        iEstPosBufferX_da[iPosBufferIndex_s32] = pEstPosX_d;
        iEstPosBufferY_da[iPosBufferIndex_s32] = pEstPosY_d;
        iPosBufferIndex_s32++; 
    } else {
        memcpy( &(iMeasPosBufferX_da[0]), &(iMeasPosBufferX_da[1]), sizeof(iMeasPosBufferX_da)-1 );
        iMeasPosBufferX_da[iPosBufferIndex_s32 - 1] = pMeasPosX_d;
        memcpy( &(iMeasPosBufferY_da[0]), &(iMeasPosBufferY_da[1]), sizeof(iMeasPosBufferY_da)-1 );
        iMeasPosBufferY_da[iPosBufferIndex_s32 - 1] = pMeasPosY_d;
        memcpy( &(iEstPosBufferX_da[0]), &(iEstPosBufferX_da[1]), sizeof(iEstPosBufferX_da)-1 );
        iEstPosBufferX_da[iPosBufferIndex_s32 - 1] = pEstPosX_d;
        memcpy( &(iEstPosBufferY_da[0]), &(iEstPosBufferY_da[1]), sizeof(iEstPosBufferY_da)-1 );
        iEstPosBufferY_da[iPosBufferIndex_s32 - 1] = pEstPosY_d;

        estimate();
    }
}

double cOrientationEstimation::calculateBearing(double pPrevX_d, double pPrevY_d, double pX_d, double pY_d) {
    double lTmpBearing_d = INVALID_ORIENTATION;
    if ((pX_d == pPrevX_d) && (pPrevY_d == pY_d)) {
        pPrevX_d += 0.3;
    }
    if ((pPrevX_d == pX_d) && (pPrevY_d > pY_d)){
        lTmpBearing_d = 6.2831853071795865;
    } else if ((pPrevX_d > pX_d) && (pPrevY_d == pY_d)){
        lTmpBearing_d = 1.57079633;
    } else if ((pPrevX_d == pX_d) && (pPrevY_d > pY_d)){
        lTmpBearing_d = 4.71238898;
    }else if ((pPrevX_d < pX_d) && (pPrevY_d == pY_d)){
        lTmpBearing_d = 3.14159265;       
    } else {
        lTmpBearing_d = atan2(pX_d - pPrevX_d, pPrevY_d - pY_d) - 1.57079633;
    }
    if(lTmpBearing_d < 0){
        lTmpBearing_d = lTmpBearing_d + 2*3.14159265;
    }
    return lTmpBearing_d;
}

double cOrientationEstimation::bearingDiffCalc(double pBearing1_d, double pBearing2_d) {
    if ((pBearing1_d == INVALID_ORIENTATION) || (pBearing2_d == INVALID_ORIENTATION)) {
        return INVALID_ORIENTATION;
    } else {
        return (3.14159265 - abs(abs(pBearing1_d - pBearing2_d) - 3.14159265)); 
    }
}

void cOrientationEstimation::estimate() {
    double lTmpBearing_d = INVALID_ORIENTATION;

    if (iPosBufferIndex_s32 == POS_BUFFER_SIZE){
        //ROS_INFO_STREAM("ROS::OR  ");
        /*for (int i = 0; i < (POS_BUFFER_SIZE - 1); i++) {
            iMeasOriBuffer_da[i] = calculateBearing(iMeasPosBufferX_da[0] - iMeasPosBufferX_da[0], iMeasPosBufferY_da[0] - iMeasPosBufferY_da[0], iMeasPosBufferX_da[i+1] - iMeasPosBufferX_da[0], iMeasPosBufferY_da[i+1] - iMeasPosBufferY_da[0]);
            iEstOriBuffer_da[i] = calculateBearing(iEstPosBufferX_da[0] - iEstPosBufferX_da[0], iEstPosBufferY_da[0] - iEstPosBufferY_da[0], iEstPosBufferX_da[i+1] - iEstPosBufferX_da[0], iEstPosBufferY_da[i+1] - iEstPosBufferY_da[0]);
            //iMeasOriBuffer_da[i] = calculateBearing(iMeasPosBufferX_da[i] - iMeasPosBufferX_da[0], iMeasPosBufferY_da[i] - iMeasPosBufferY_da[0], iMeasPosBufferX_da[i+1] - iMeasPosBufferX_da[0], iMeasPosBufferY_da[i+1] - iMeasPosBufferY_da[0]);
            //iEstOriBuffer_da[i] = calculateBearing(iEstPosBufferX_da[i] - iEstPosBufferX_da[0], iEstPosBufferY_da[i] - iEstPosBufferY_da[0], iEstPosBufferX_da[i+1] - iEstPosBufferX_da[0], iEstPosBufferY_da[i+1] - iEstPosBufferY_da[0]);
            iMeasEstOriBufferDif_da[i] = bearingDiffCalc(iMeasOriBuffer_da[i], iEstOriBuffer_da[i]);
        }
        iEstOriBuffer_da[iPosBufferIndex_s32-1] = iEstOriBuffer_da[iPosBufferIndex_s32-2]; 
        iMeasOriBuffer_da[iPosBufferIndex_s32-1] = iMeasOriBuffer_da[iPosBufferIndex_s32-2]; 
        iMeasEstOriBufferDif_da[iPosBufferIndex_s32-1] = iMeasEstOriBufferDif_da[iPosBufferIndex_s32-2];
        filterOriDiff();
        idMeanDifference_d = iMeasEstOriBufferDif_da[iPosBufferIndex_s32-2];*/

        
        iMeasOriBuffer_da[iMeasEstOriBufferDifIndex_s32] = calculateBearing(0, 0, iMeasPosBufferX_da[POS_BUFFER_SIZE-1] - iMeasPosBufferX_da[0], iMeasPosBufferY_da[POS_BUFFER_SIZE - 1] - iMeasPosBufferY_da[0]);
        iEstOriBuffer_da[iMeasEstOriBufferDifIndex_s32] = calculateBearing(0, 0, iEstPosBufferX_da[POS_BUFFER_SIZE-1] - iEstPosBufferX_da[0], iEstPosBufferY_da[POS_BUFFER_SIZE - 1] - iEstPosBufferY_da[0]);
        iMeasEstOriBufferDif_da[iMeasEstOriBufferDifIndex_s32] = bearingDiffCalc(iMeasOriBuffer_da[iMeasEstOriBufferDifIndex_s32], iEstOriBuffer_da[iMeasEstOriBufferDifIndex_s32]);

        if (iMeasEstOriBufferDifIndex_s32 >= ORIENTATION_DIF_BUFFER_SIZE-1) {
            iMeasEstOriBufferDifValid_b = true;
            iMeasEstOriBufferDifIndex_s32 = 0;
        } else {
            iMeasEstOriBufferDifIndex_s32++;
        }
        
        if (iMeasEstOriBufferDifValid_b == true) {
            filterOriDiff2();
            filterOri();
        }
    }
}

void cOrientationEstimation::filterOri() {
    double lMean_d = 0;
    double lSuMeast_d = 0;
    double lSumMeas_d = 0;

    if (iPosBufferIndex_s32 == POS_BUFFER_SIZE){
        for (int i = 0; i < (POS_BUFFER_SIZE); i++) {
            lSumMeas_d += iMeasOriBuffer_da[i];
            lSuMeast_d += iEstOriBuffer_da[i];
        }

        iFiltMeasOri_d  = lSumMeas_d / POS_BUFFER_SIZE;
        iFiltEstOri_d  = lSuMeast_d / POS_BUFFER_SIZE;
    }
}

void cOrientationEstimation::filterOriDiff() {
    double lMean_d = 0;
    double lSum_d = 0;
    double lSumFiltered_d = 0;
    double lVariance_d = 0;
    double lStd_d = 0;
    double lLowerLimit_d = 0;
    double lUpperLimit_d = 0;

    if (iMeasEstOriBufferDifValid_b == true){
        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lSum_d += iMeasEstOriBufferDif_da[i];
        }

        lMean_d = lSum_d / ORIENTATION_DIF_BUFFER_SIZE;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lVariance_d += pow((iMeasEstOriBufferDif_da[i] - lMean_d), 2);
        }

        lVariance_d = lVariance_d * (1 / (ORIENTATION_DIF_BUFFER_SIZE));

        lStd_d = sqrt(lVariance_d);

        lUpperLimit_d = lMean_d + 2*lStd_d;
        lLowerLimit_d = lMean_d - 2*lStd_d;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE-4); i++) {
            if ((iMeasEstOriBufferDif_da[i] > lUpperLimit_d) && (iMeasEstOriBufferDif_da[i + 1] > lUpperLimit_d) && (iMeasEstOriBufferDif_da[i + 2] > lUpperLimit_d) && (iMeasEstOriBufferDif_da[i + 3] > lUpperLimit_d)){
                for (int j = 0; j < 4; j++) {
                    iFiltMeasEstOriBufferDif_da[i+j] = lMean_d + lStd_d;
                    lSumFiltered_d += iFiltMeasEstOriBufferDif_da[i+j];
                }
                i += 4;
            } else {
                iFiltMeasEstOriBufferDif_da[i] = iMeasEstOriBufferDif_da[i];
                lSumFiltered_d += iFiltMeasEstOriBufferDif_da[i];
            }
        }
        iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-1] = iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-2] = iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-3] = iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-4] = iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        lSumFiltered_d += (4 * iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5]);
        iFilteredMeanDifference_d = lSumFiltered_d / ORIENTATION_DIF_BUFFER_SIZE;
        iOrientationIsValid_b = true;
    }
}


void cOrientationEstimation::filterOriDiff2() {
    double lMean_d = 0;
    double lSum_d = 0;
    double lSumFiltered_d = 0;
    double lVariance_d = 0;
    double lStd_d = 0;
    double lLowerLimit_d = 0;
    double lUpperLimit_d = 0;

    if (iMeasEstOriBufferDifValid_b == true){
        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lSum_d += iMeasEstOriBufferDif_da[i];
        }

        lMean_d = lSum_d / ORIENTATION_DIF_BUFFER_SIZE;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lVariance_d += pow((iMeasEstOriBufferDif_da[i] - lMean_d), 2);
        }

        lVariance_d = lVariance_d * (1 / (ORIENTATION_DIF_BUFFER_SIZE)); //(1 / (ORIENTATION_DIF_BUFFER_SIZE - 1));

        lStd_d = sqrt(lVariance_d);

        lUpperLimit_d = lMean_d + 2*lStd_d;
        lLowerLimit_d = lMean_d - 2*lStd_d;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            if (iMeasEstOriBufferDif_da[i] > lUpperLimit_d) {
                iMeasEstOriBufferDif_da[i] = lMean_d + lStd_d;
                iFiltMeasEstOriBufferDif_da[i] = lMean_d + lStd_d;
            }
            else if (iMeasEstOriBufferDif_da[i] < lLowerLimit_d) {
                iMeasEstOriBufferDif_da[i] = lMean_d - lStd_d;
                iFiltMeasEstOriBufferDif_da[i] = lMean_d - lStd_d;
            } else {
                iFiltMeasEstOriBufferDif_da[i] = iMeasEstOriBufferDif_da[i];
            }
            lSumFiltered_d += iFiltMeasEstOriBufferDif_da[i];
        }
        iFilteredMeanDifference_d = lSumFiltered_d / ORIENTATION_DIF_BUFFER_SIZE;
        iOrientationIsValid_b = true;
    }
}

