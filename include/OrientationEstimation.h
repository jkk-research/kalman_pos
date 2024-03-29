#pragma once
#ifndef ORIENTATION_ESTIMATION
#define ORIENTATION_ESTIMATION

#define POS_BUFFER_SIZE 5
#define ORIENTATION_DIF_BUFFER_SIZE 5
#define INVALID_ORIENTATION -999999

class cOrientationEstimation {
    private:
        double iMeasPosBufferX_da[POS_BUFFER_SIZE];
        double iMeasPosBufferY_da[POS_BUFFER_SIZE];
        double iEstPosBufferX_da[POS_BUFFER_SIZE];
        double iEstPosBufferY_da[POS_BUFFER_SIZE];
        double iMeasOriBuffer_da[ORIENTATION_DIF_BUFFER_SIZE];
        double iEstOriBuffer_da[ORIENTATION_DIF_BUFFER_SIZE];
        double iMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE];
        double iFiltMeasEstOriBufferDif_da[ORIENTATION_DIF_BUFFER_SIZE];
        int iPosBufferIndex_s32 = 0;
        int iMeasEstOriBufferDifIndex_s32 = 0;
        bool iMeasEstOriBufferDifValid_b= false;

        void init();
        void estimate();
        double calculateBearing(double pPrevX_d, double pPrevY_d, double pX_d, double pY_d);
        double bearingDiffCalc(double pBearing1_d, double pBearing2_d);
        void filterOriDiff();
        void filterOriDiff2();
        void filterOri();
    public:
        double iFilteredMeanDifference_d;
        double iFiltMeasOri_d;
        double iFiltEstOri_d;
        bool iOrientationIsValid_b = false;
        cOrientationEstimation();
        ~cOrientationEstimation();
        void addPosition(double pMeasPosX_d, double pMeasPosY_d, double pEstPosX_d, double pEstPosY_d);
};

#endif