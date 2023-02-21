#include "KinematicVehicleModelEKFwoGNSS.h"
#include "MatrixInverse.hpp"

#include <math.h>

double kinEKFwoGNSSLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = pPrevMeasuredValues_s.longitudinalAcceleration_d * pTs_d + pPrevModelStates_s.longitudinalVelocity_d;

	return lReturnValue_d;
}

double kinEKFwoGNSSLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = pPrevMeasuredValues_s.lateralAcceleration_d * pTs_d + pPrevModelStates_s.lateralVelocity_d;

	return lReturnValue_d;
}

double kinEKFwoGNSSBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		atan(tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)));

	return lReturnValue_d;
}

double kinEKFwoGNSSYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, double pLongitudinalVelocity_d, double pBeta_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = pLongitudinalVelocity_d * tan(pBeta_d) / pVehicleParameters_s.l2_d;

	return lReturnValue_d;
}

double kinEKFwoGNSSMeasBasedLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

double kinEKFwoGNSSMeasBasedLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));
	return lLateralSpeed_d;
}

double kinEKFwoGNSSLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	return pMeasuredValues_s.lateralAcceleration_d;
}

double kinEKFwoGNSSYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.yawRate_d * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

double kinEKFwoGNSSPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevLongitudinalSpeed_d = 0;
	double lPrevLateralSpeed_d = 0;

	lPrevLongitudinalSpeed_d = pPrevModelStates_s.longitudinalVelocity_d;
	lPrevLateralSpeed_d = pPrevModelStates_s.lateralVelocity_d;

	lReturnValue_d =
		pPrevModelStates_s.positionX_d
		+
		(cos(pPrevModelStates_s.yawAngle_d) * pTs_d *
			lPrevLongitudinalSpeed_d)
		-
		(sin(pPrevModelStates_s.yawAngle_d) * pTs_d *
			lPrevLateralSpeed_d);

	return lReturnValue_d;
}

double kinEKFwoGNSSPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevLongitudinalSpeed_d = 0;
	double lPrevLateralSpeed_d = 0;

	lPrevLongitudinalSpeed_d = pPrevModelStates_s.longitudinalVelocity_d;
	lPrevLateralSpeed_d = pPrevModelStates_s.lateralVelocity_d;

	lReturnValue_d =
		pPrevModelStates_s.positionY_d
		+
		(sin(pPrevModelStates_s.yawAngle_d) * pTs_d *
			lPrevLongitudinalSpeed_d)
		+
		(cos(pPrevModelStates_s.yawAngle_d) * pTs_d *
			lPrevLateralSpeed_d);

	return lReturnValue_d;
}

void kinEKFwoGNSSEstimate(sModelStates &pOutModelStates_s, matrix<double>& pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double>& pPrevP_m, matrix<double>& pQ_m, matrix<double>& pR_m) {
	double lYawAngle_d = kinEKFwoGNSSYawAngleCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPrevYawAngle_d = pPrevModelStates_s.yawAngle_d;
	double lPrevLongitudinalVelocity_d = pPrevModelStates_s.longitudinalVelocity_d;
	double lPrevLateralVelocity_d = pPrevModelStates_s.lateralVelocity_d;
	double lLongitudinalVelocity_d = kinEKFwoGNSSLongitudinalVelocityCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lLateralVelocity_d = kinEKFwoGNSSLateralVelocityCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lBeta_d = kinEKFwoGNSSBetaCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lYawRate_d = kinEKFwoGNSSYawRateCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d, lLongitudinalVelocity_d, lBeta_d);
	double lLateralAcc_d = kinEKFwoGNSSLateralAccCalculation(pVehicleParameters_s, pMeasuredValues_s, lBeta_d, lYawRate_d);
	double lPositionX_d = kinEKFwoGNSSPositionXCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPositionY_d = kinEKFwoGNSSPositionYCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lMeasBasedLateralVelocity_d = kinEKFwoGNSSMeasBasedLateralVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	double lMeasBasedLongitudinalVelocity_d = kinEKFwoGNSSMeasBasedLongitudinalVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);

	double lMeasYawRate_d = pPrevMeasuredValues_s.yawRate_d;
	double lMeasYawAngle_d = pPrevMeasuredValues_s.yawAngle_d;
	double lMeasLateralAcc_d = pPrevMeasuredValues_s.lateralAcceleration_d;
	double lMeasPositionX_d = pPrevMeasuredValues_s.positionX_d;
	double lMeasPositionY_d = pPrevMeasuredValues_s.positionY_d;
	double lPrevMeasVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;
	double lPrevMeasLateralAcc_d = pPrevMeasuredValues_s.lateralAcceleration_d;


	vector<double> lh_v(3);
	vector<double> lxPre_v(5);
	vector<double> lxPro_v(5);
	vector<double> ly_v(3);

	matrix<double> lPPre_m(5, 5);
	matrix<double> lK_m(5, 5);
	matrix<double> lF_m(5, 5);
	matrix<double> lH_m(3, 5);
	matrix<double> lL_m(5, 5);
	matrix<double> lM_m(3, 3);
	matrix<double> lI_m(5, 5);

	lPrevMeasVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;

	lh_v(0) = lLongitudinalVelocity_d;
	lh_v(1) = lLateralVelocity_d;
	lh_v(2) = lYawRate_d;

	lxPre_v(0) = lLongitudinalVelocity_d;
	lxPre_v(1) = lLateralVelocity_d;
	lxPre_v(2) = lPositionX_d;
	lxPre_v(3) = lPositionY_d;
	lxPre_v(4) = lYawAngle_d;

	ly_v(0) = lMeasBasedLongitudinalVelocity_d;
	ly_v(1) = lMeasBasedLateralVelocity_d;
	ly_v(2) = lMeasYawRate_d;
	//ly_v(0) = lMeasBasedLongitudinalVelocity_d;
	//ly_v(1) = lMeasBasedLateralVelocity_d;
	//ly_v(2) = lMeasPositionX_d;
	//ly_v(3) = lMeasPositionY_d;
	//ly_v(4) = lMeasYawRate_d;

	lL_m(0, 0) = 1;
	lL_m(0, 1) = 0;
	lL_m(0, 2) = 0;
	lL_m(0, 3) = 0;
	lL_m(0, 4) = 0;

	lL_m(1, 0) = 0;
	lL_m(1, 1) = 1;
	lL_m(1, 2) = 0;
	lL_m(1, 3) = 0;
	lL_m(1, 4) = 0;

	lL_m(2, 0) = 0;
	lL_m(2, 1) = 0;
	lL_m(2, 2) = 1;
	lL_m(2, 3) = 0;
	lL_m(2, 4) = 0;

	lL_m(3, 0) = 0;
	lL_m(3, 1) = 0;
	lL_m(3, 2) = 0;
	lL_m(3, 3) = 1;
	lL_m(3, 4) = 0;

	lL_m(4, 0) = 0;
	lL_m(4, 1) = 0;
	lL_m(4, 2) = 0;
	lL_m(4, 3) = 0;
	lL_m(4, 4) = 1;

	lM_m(0, 0) = 1;
	lM_m(0, 1) = 0;
	lM_m(0, 2) = 0;

	lM_m(1, 0) = 0;
	lM_m(1, 1) = 1;
	lM_m(1, 2) = 0;

	lM_m(2, 0) = 0;
	lM_m(2, 1) = 0;
	lM_m(2, 2) = 1;

	lI_m(0, 0) = 1;
	lI_m(0, 1) = 0;
	lI_m(0, 2) = 0;
	lI_m(0, 3) = 0;
	lI_m(0, 4) = 0;

	lI_m(1, 0) = 0;
	lI_m(1, 1) = 1;
	lI_m(1, 2) = 0;
	lI_m(1, 3) = 0;
	lI_m(1, 4) = 0;

	lI_m(2, 0) = 0;
	lI_m(2, 1) = 0;
	lI_m(2, 2) = 1;
	lI_m(2, 3) = 0;
	lI_m(2, 4) = 0;

	lI_m(3, 0) = 0;
	lI_m(3, 1) = 0;
	lI_m(3, 2) = 0;
	lI_m(3, 3) = 1;
	lI_m(3, 4) = 0;

	lI_m(4, 0) = 0;
	lI_m(4, 1) = 0;
	lI_m(4, 2) = 0;
	lI_m(4, 3) = 0;
	lI_m(4, 4) = 1;

	lF_m(0, 0) = 1;
	lF_m(0, 1) = 0;
	lF_m(0, 2) = 0;
	lF_m(0, 3) = 0;
	lF_m(0, 4) = 0;

	lF_m(1, 0) = 0;
	lF_m(1, 1) = 1;
	lF_m(1, 2) = 0;
	lF_m(1, 3) = 0;
	lF_m(1, 4) = 0;

	lF_m(2, 0) = pTs_d * cos(lPrevYawAngle_d);
	lF_m(2, 1) = 0;
	lF_m(2, 2) = 1;
	lF_m(2, 3) = 0;
	lF_m(2, 4) = pTs_d * (-lPrevLongitudinalVelocity_d) * sin(lPrevYawAngle_d);

	lF_m(3, 0) = 0;
	lF_m(3, 1) = pTs_d * sin(lPrevYawAngle_d);
	lF_m(3, 2) = 0;
	lF_m(3, 3) = 1;
	lF_m(3, 4) = pTs_d * (lPrevLateralVelocity_d)*cos(lPrevYawAngle_d);

	lF_m(4, 0) = 0;
	lF_m(4, 1) = 0;
	lF_m(4, 2) = 0;
	lF_m(4, 3) = 0;
	lF_m(4, 4) = 1;

	lH_m(0, 0) = 1;
	lH_m(0, 1) = 0;
	lH_m(0, 2) = 0;
	lH_m(0, 3) = 0;
	lH_m(0, 4) = 0;

	lH_m(1, 0) = 0;
	lH_m(1, 1) = 1;
	lH_m(1, 2) = 0;
	lH_m(1, 3) = 0;
	lH_m(1, 4) = 0;

	// TODO: tan or sin (it have to be checked)?
	lH_m(2, 0) = tan(lBeta_d) / pVehicleParameters_s.l2_d;
	lH_m(2, 1) = 0;
	lH_m(2, 2) = 0;
	lH_m(2, 3) = 0;
	lH_m(2, 4) = 0;

	matrix<double> ltmpFMultPrevP_m(5, 5);
	matrix<double> ltmpLMultQ_m(5, 5);
	matrix<double> ltmpHMultPPre_m(3, 5);
	matrix<double> ltmpMMultR_m(3, 3);
	matrix<double> ltmpPPreMultHT_m(5, 3);
	matrix<double> ltmpMMultRMultMT_m(3, 3);
	matrix<double> ltmpHMultPPreMultHT_m(3, 3);
	matrix<double> ltmpSumMatrix_m(3, 3);
	matrix<double> ltmpInvMatrix_m(3, 3);

	ltmpFMultPrevP_m = prec_prod(lF_m, pPrevP_m);
	ltmpLMultQ_m = prec_prod(lL_m, pQ_m);
	ltmpMMultR_m = prec_prod(lM_m, pR_m);
	lPPre_m = prec_prod(ltmpFMultPrevP_m, trans(lF_m)) + prec_prod(ltmpLMultQ_m, trans(lL_m));
	ltmpHMultPPre_m = prec_prod(lH_m, lPPre_m);
	ltmpPPreMultHT_m = prec_prod(lPPre_m, trans(lH_m));
	ltmpMMultRMultMT_m = prec_prod(ltmpMMultR_m, trans(lM_m));
	ltmpHMultPPreMultHT_m = prec_prod(ltmpHMultPPre_m, trans(lH_m));
	ltmpSumMatrix_m = ltmpHMultPPreMultHT_m + ltmpMMultRMultMT_m;
	InvertMatrix(ltmpSumMatrix_m, ltmpInvMatrix_m);
	lK_m = prec_prod(ltmpPPreMultHT_m, ltmpInvMatrix_m);
	lxPro_v = lxPre_v + prec_prod(lK_m, (ly_v - lh_v));

	pOutModelStates_s.beta_d = lBeta_d;
	pOutModelStates_s.yawRate_d = lYawRate_d;
	pOutModelStates_s.yawAngle_d = lxPro_v(4);
	pOutModelStates_s.positionX_d = lxPro_v(2);
	pOutModelStates_s.positionY_d = lxPro_v(3);
	pOutModelStates_s.lateralAcceleration_d = lLateralAcc_d;
	pOutModelStates_s.longitudinalVelocity_d = lxPro_v(0);
	pOutModelStates_s.lateralVelocity_d = lxPro_v(1);

	pOutP_m = prec_prod((lI_m - prec_prod(lK_m, lH_m)), lPPre_m);
}
