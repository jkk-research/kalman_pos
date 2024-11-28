#pragma once
#ifndef POSITION_ESTIMATION
#define POSITION_ESTIMATION

#include "CombinedVehicleModel.h"
#include "OrientationEstimation.h"

class cPositionEstimation {
    private:
        cCombinedVehicleModel iCombinedVehicleModel_cl;
        cOrientationEstimation iOrientationEstimation_cl;
        bool iOriEstimationEnabled_b = false;
        bool iFirstIteration_b = true;
        int iLoopRateHz_i32 = 20;
        double iTs_d = 0;
        bool iDynamicTimeCalcEnabled_b = false;
        unsigned long long iPrevMillisecondsSinceEpoch_u64 = 0;
        unsigned long long iMillisecondsSinceEpoch_u64 = 0;
        double iPrevGNSSMeasPosX_d = 0;
        double iPrevGNSSMeasPosY_d = 0;
        double iPrevSLAMMeasPosX_d = 0;
        double iPrevSLAMMeasPosY_d = 0;
        double iPrevEstPosX_d = 0;
        double iPrevEstPosY_d = 0;
        bool iPrevOrientationIsValid_b = false;
        double iKinSpeedLimit_d = 5;
        double iDefaultKinSpeedLimit_d = 5;
        double iTravDistanceOdom_d = 0;
        double iTravDistanceEstPos_d = 0;
        int iAccuracyScaleFactor_d = 10;
        
    private:
        void cycleTimeCalculation(void);
        void traveledDistanceCalculation(void);

    public:
        cPositionEstimation();
        cPositionEstimation(bool pDynamicTimeCalcEnabled_b, int pLoopRateHz_i32, sVehicleParameters &pVehicleParameters_s, float pKinematicModelMaxSpeed_f, bool pOriEstimationEnabled_b);
        ~cPositionEstimation();
        void initEstimation(bool pDynamicTimeCalcEnabled_b, int pLoopRateHz_i32, sVehicleParameters &pVehicleParameters_s, float pKinematicModelMaxSpeed_f, bool pOriEstimationEnabled_b);
        void setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d);
        void setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d);
        void setMeasuredValuesSLAM(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d);
        void setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d);
        void iterateEstimation(bool pUseRawModel_b, bool pGNSSAvailable_b, bool pSLAMAvailable_b, double pGNSSCovariance_da[3], double pSLAMCovariance_da[3], bool pReset_b);
        void getModelStates(sModelStates* pOutModelStates_s);
        double getCogDistanceFromBaselinkX(void);
        double getCogDistanceFromBaselinkY(void);
        double getCogDistanceFromBaselinkZ(void);
        double getAccuracyScaleFactor(void);
        double getFiltMeasOri(void);
        double getTravDistanceOdom(void);
        double getTravDistanceEstPos(void);
};
#endif