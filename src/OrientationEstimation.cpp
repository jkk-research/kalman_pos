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
        iMesPosBufferX_da[i] = 0;
        iMesPosBufferY_da[i] = 0;
        iEstPosBufferX_da[i] = 0;
        iEstPosBufferY_da[i] = 0;
    }

    for (int i = 0; i < ORIENTATION_DIF_BUFFER_SIZE; i++) {
        iMesOriBuffer_da[i] = INVALID_ORIENTATION;
        iEstOriBuffer_da[i] = INVALID_ORIENTATION;
        iMesEstOriBufferDif_da[i] = INVALID_ORIENTATION;
        iFiltMesEstOriBufferDif_da[i] = INVALID_ORIENTATION;
    }

    iFilteredMeanDifference_d = INVALID_ORIENTATION;
    iFiltMesOri_d = INVALID_ORIENTATION;
    iFiltEstOri_d = INVALID_ORIENTATION;
    iMesEstOriBufferDifValid_b= false;
    iPosBufferIndex_s32 = 0;
    iMesEstOriBufferDifIndex_s32 = 0;
    iOrientationIsValid_b = false;
}

void cOrientationEstimation::addPosition(double pMesPosX_d, double pMesPosY_d, double pEstPosX_d, double pEstPosY_d){
    if (iPosBufferIndex_s32 < POS_BUFFER_SIZE) {
        iMesPosBufferX_da[iPosBufferIndex_s32] = pMesPosX_d;
        iMesPosBufferY_da[iPosBufferIndex_s32] = pMesPosY_d;
        iEstPosBufferX_da[iPosBufferIndex_s32] = pEstPosX_d;
        iEstPosBufferY_da[iPosBufferIndex_s32] = pEstPosY_d;
        iPosBufferIndex_s32++; 
    } else {
        memcpy( &(iMesPosBufferX_da[0]), &(iMesPosBufferX_da[1]), sizeof(iMesPosBufferX_da)-1 );
        iMesPosBufferX_da[iPosBufferIndex_s32 - 1] = pMesPosX_d;
        memcpy( &(iMesPosBufferY_da[0]), &(iMesPosBufferY_da[1]), sizeof(iMesPosBufferY_da)-1 );
        iMesPosBufferY_da[iPosBufferIndex_s32 - 1] = pMesPosY_d;
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
            iMesOriBuffer_da[i] = calculateBearing(iMesPosBufferX_da[0] - iMesPosBufferX_da[0], iMesPosBufferY_da[0] - iMesPosBufferY_da[0], iMesPosBufferX_da[i+1] - iMesPosBufferX_da[0], iMesPosBufferY_da[i+1] - iMesPosBufferY_da[0]);
            iEstOriBuffer_da[i] = calculateBearing(iEstPosBufferX_da[0] - iEstPosBufferX_da[0], iEstPosBufferY_da[0] - iEstPosBufferY_da[0], iEstPosBufferX_da[i+1] - iEstPosBufferX_da[0], iEstPosBufferY_da[i+1] - iEstPosBufferY_da[0]);
            //iMesOriBuffer_da[i] = calculateBearing(iMesPosBufferX_da[i] - iMesPosBufferX_da[0], iMesPosBufferY_da[i] - iMesPosBufferY_da[0], iMesPosBufferX_da[i+1] - iMesPosBufferX_da[0], iMesPosBufferY_da[i+1] - iMesPosBufferY_da[0]);
            //iEstOriBuffer_da[i] = calculateBearing(iEstPosBufferX_da[i] - iEstPosBufferX_da[0], iEstPosBufferY_da[i] - iEstPosBufferY_da[0], iEstPosBufferX_da[i+1] - iEstPosBufferX_da[0], iEstPosBufferY_da[i+1] - iEstPosBufferY_da[0]);
            iMesEstOriBufferDif_da[i] = bearingDiffCalc(iMesOriBuffer_da[i], iEstOriBuffer_da[i]);
        }
        iEstOriBuffer_da[iPosBufferIndex_s32-1] = iEstOriBuffer_da[iPosBufferIndex_s32-2]; 
        iMesOriBuffer_da[iPosBufferIndex_s32-1] = iMesOriBuffer_da[iPosBufferIndex_s32-2]; 
        iMesEstOriBufferDif_da[iPosBufferIndex_s32-1] = iMesEstOriBufferDif_da[iPosBufferIndex_s32-2];
        filterOriDiff();
        idMeanDifference_d = iMesEstOriBufferDif_da[iPosBufferIndex_s32-2];*/

        
        iMesOriBuffer_da[iMesEstOriBufferDifIndex_s32] = calculateBearing(0, 0, iMesPosBufferX_da[POS_BUFFER_SIZE-1] - iMesPosBufferX_da[0], iMesPosBufferY_da[POS_BUFFER_SIZE - 1] - iMesPosBufferY_da[0]);
        iEstOriBuffer_da[iMesEstOriBufferDifIndex_s32] = calculateBearing(0, 0, iEstPosBufferX_da[POS_BUFFER_SIZE-1] - iEstPosBufferX_da[0], iEstPosBufferY_da[POS_BUFFER_SIZE - 1] - iEstPosBufferY_da[0]);
        iMesEstOriBufferDif_da[iMesEstOriBufferDifIndex_s32] = bearingDiffCalc(iMesOriBuffer_da[iMesEstOriBufferDifIndex_s32], iEstOriBuffer_da[iMesEstOriBufferDifIndex_s32]);

        if (iMesEstOriBufferDifIndex_s32 >= ORIENTATION_DIF_BUFFER_SIZE-1) {
            iMesEstOriBufferDifValid_b = true;
            iMesEstOriBufferDifIndex_s32 = 0;
        } else {
            iMesEstOriBufferDifIndex_s32++;
        }
        
        if (iMesEstOriBufferDifValid_b == true) {
            filterOriDiff2();
            filterOri();
        }
    }
}

void cOrientationEstimation::filterOri() {
    double lMean_d = 0;
    double lSumEst_d = 0;
    double lSumMes_d = 0;

    if (iPosBufferIndex_s32 == POS_BUFFER_SIZE){
        for (int i = 0; i < (POS_BUFFER_SIZE); i++) {
            lSumMes_d += iMesOriBuffer_da[i];
            lSumEst_d += iEstOriBuffer_da[i];
        }

        iFiltMesOri_d  = lSumMes_d / POS_BUFFER_SIZE;
        iFiltEstOri_d  = lSumEst_d / POS_BUFFER_SIZE;
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

    if (iMesEstOriBufferDifValid_b == true){
        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lSum_d += iMesEstOriBufferDif_da[i];
        }

        lMean_d = lSum_d / ORIENTATION_DIF_BUFFER_SIZE;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lVariance_d += pow((iMesEstOriBufferDif_da[i] - lMean_d), 2);
        }

        lVariance_d = lVariance_d * (1 / (ORIENTATION_DIF_BUFFER_SIZE));

        lStd_d = sqrt(lVariance_d);

        lUpperLimit_d = lMean_d + 2*lStd_d;
        lLowerLimit_d = lMean_d - 2*lStd_d;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE-4); i++) {
            if ((iMesEstOriBufferDif_da[i] > lUpperLimit_d) && (iMesEstOriBufferDif_da[i + 1] > lUpperLimit_d) && (iMesEstOriBufferDif_da[i + 2] > lUpperLimit_d) && (iMesEstOriBufferDif_da[i + 3] > lUpperLimit_d)){
                for (int j = 0; j < 4; j++) {
                    iFiltMesEstOriBufferDif_da[i+j] = lMean_d + lStd_d;
                    lSumFiltered_d += iFiltMesEstOriBufferDif_da[i+j];
                }
                i += 4;
            } else {
                iFiltMesEstOriBufferDif_da[i] = iMesEstOriBufferDif_da[i];
                lSumFiltered_d += iFiltMesEstOriBufferDif_da[i];
            }
        }
        iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-1] = iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-2] = iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-3] = iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-4] = iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5];
        lSumFiltered_d += (4 * iFiltMesEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE-5]);
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

    if (iMesEstOriBufferDifValid_b == true){
        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lSum_d += iMesEstOriBufferDif_da[i];
        }

        lMean_d = lSum_d / ORIENTATION_DIF_BUFFER_SIZE;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            lVariance_d += pow((iMesEstOriBufferDif_da[i] - lMean_d), 2);
        }

        lVariance_d = lVariance_d * (1 / (ORIENTATION_DIF_BUFFER_SIZE)); //(1 / (ORIENTATION_DIF_BUFFER_SIZE - 1));

        lStd_d = sqrt(lVariance_d);

        lUpperLimit_d = lMean_d + 2*lStd_d;
        lLowerLimit_d = lMean_d - 2*lStd_d;

        for (int i = 0; i < (ORIENTATION_DIF_BUFFER_SIZE); i++) {
            if (iMesEstOriBufferDif_da[i] > lUpperLimit_d) {
                iMesEstOriBufferDif_da[i] = lMean_d + lStd_d;
                iFiltMesEstOriBufferDif_da[i] = lMean_d + lStd_d;
            }
            else if (iMesEstOriBufferDif_da[i] < lLowerLimit_d) {
                iMesEstOriBufferDif_da[i] = lMean_d - lStd_d;
                iFiltMesEstOriBufferDif_da[i] = lMean_d - lStd_d;
            } else {
                iFiltMesEstOriBufferDif_da[i] = iMesEstOriBufferDif_da[i];
            }
            lSumFiltered_d += iFiltMesEstOriBufferDif_da[i];
        }
        iFilteredMeanDifference_d = lSumFiltered_d / ORIENTATION_DIF_BUFFER_SIZE;
        iOrientationIsValid_b = true;
    }
}

