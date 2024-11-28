#include "DynamicVehicleModel_wEKFwExtPos1.h"
#include "MatrixInverse.hpp"
#include "rclcpp/rclcpp.hpp"

#include <math.h>

double dyn_wEKFwExtPos1_LongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

double dyn_wEKFwExtPos1_LateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d));
	return lLateralSpeed_d;
}

double dyn_wEKFwExtPos1_BetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	//if (cos(pPrevModelStates_s.iBeta_d) != 0) {
	//	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d / cos(pPrevModelStates_s.iBeta_d); // vx / cos(beta)
	//}
	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;

	if ((lPrevVehicleSpeed_d != 0) && (((pVehicleParameters_s.iM_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d)) - 1) != 0)) {
		lReturnValue_d =
			(pPrevMeasuredValues_s.iSteeringAngle_d *
				pTs_d * pVehicleParameters_s.iC1_d / (pVehicleParameters_s.iM_d * lPrevVehicleSpeed_d))
			+
			((1 - pTs_d * ((pVehicleParameters_s.iC1_d + pVehicleParameters_s.iC2_d) / (pVehicleParameters_s.iM_d * lPrevVehicleSpeed_d))) *
				pPrevModelStates_s.iBeta_d)

			+
			((((pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d - pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d) / (pVehicleParameters_s.iM_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d)) - 1) * pTs_d) *
				pPrevModelStates_s.iYawRate_d);
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;
}

double dyn_wEKFwExtPos1_YawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	//if (cos(pPrevModelStates_s.iBeta_d) != 0) {
	//	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d / cos(pPrevModelStates_s.iBeta_d); // vx / cos(beta)
	//}
	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;

	if (lPrevVehicleSpeed_d != 0) {
		lReturnValue_d =
			pPrevModelStates_s.iBeta_d * pTs_d * (pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d - pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d) / pVehicleParameters_s.iJz_d
			+
			pPrevModelStates_s.iYawRate_d *
			(1 - pTs_d * (pVehicleParameters_s.iC2_d * (pVehicleParameters_s.iL2_d * pVehicleParameters_s.iL2_d) + pVehicleParameters_s.iC1_d * (pVehicleParameters_s.iL1_d * pVehicleParameters_s.iL1_d)) / (lPrevVehicleSpeed_d * pVehicleParameters_s.iJz_d))
			+
			pPrevMeasuredValues_s.iSteeringAngle_d * pTs_d * pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d / pVehicleParameters_s.iJz_d;
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;
}

double dyn_wEKFwExtPos1_LateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	double lVehicleSpeed_d = 0;
	double lReturnValue_d = 0;

	//if (cos(pBeta_d) != 0) {
	//	lVehicleSpeed_d = pMeasuredValues_s.iVehicleSpeed_d / cos(pBeta_d); // vx / cos(beta)
	//}
	lVehicleSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;

	if (lVehicleSpeed_d != 0) {
		lReturnValue_d =
			((-pBeta_d) * (pVehicleParameters_s.iC1_d + pVehicleParameters_s.iC2_d) / pVehicleParameters_s.iM_d)
			+
			(pYawRate_d * (pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d - pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d)) /
			(lVehicleSpeed_d * pVehicleParameters_s.iM_d)
			+
			pMeasuredValues_s.iSteeringAngle_d * (pVehicleParameters_s.iC1_d / pVehicleParameters_s.iM_d);
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;
}

double dyn_wEKFwExtPos1_YawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.iYawRate_d * pTs_d) + pPrevModelStates_s.iYawAngle_d;

	return lReturnValue_d;
}

double dyn_wEKFwExtPos1_PositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pPrevModelStates_s.iPositionX_d
		+
		(cos(pPrevModelStates_s.iYawAngle_d) *
			pPrevMeasuredValues_s.iVehicleSpeed_d * pTs_d)
		-
		(sin(pPrevModelStates_s.iYawAngle_d) *
			pPrevModelStates_s.iLateralAcceleration_d * (pTs_d * pTs_d / 2));

	return lReturnValue_d;
}

double dyn_wEKFwExtPos1_PositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pPrevModelStates_s.iPositionY_d
		+
		(sin(pPrevModelStates_s.iYawAngle_d) *
			pPrevMeasuredValues_s.iVehicleSpeed_d * pTs_d)
		+
		(cos(pPrevModelStates_s.iYawAngle_d) *
			pPrevModelStates_s.iLateralAcceleration_d * (pTs_d * pTs_d / 2));

	return lReturnValue_d;
}

void dyn_wEKFwExtPos1_Estimate(sModelStates &pOutModelStates_s, matrix<double> &pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double> &pPrevP_m, matrix<double> &pQ_m, matrix<double> &pR_m) {
	double lYawRate_d			    = dyn_wEKFwExtPos1_YawRateCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lYawAngle_d			    = dyn_wEKFwExtPos1_YawAngleCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPrevYawAngle_d		    = pPrevModelStates_s.iYawAngle_d;
	double lBeta_d				    = dyn_wEKFwExtPos1_BetaCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lLateralAcc_d		    = dyn_wEKFwExtPos1_LateralAccCalculation(pVehicleParameters_s, pMeasuredValues_s, lBeta_d, lYawRate_d);
	double lPositionX_d			    = dyn_wEKFwExtPos1_PositionXCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPositionY_d			    = dyn_wEKFwExtPos1_PositionYCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lLateralVelocity_d       = dyn_wEKFwExtPos1_LateralVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	double lLongitudinalVelocity_d  = dyn_wEKFwExtPos1_LongitudinalVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	double lMeasYawRate_d		    = pPrevMeasuredValues_s.iYawRate_d;
	double lMeasYawAngle_d		    = pPrevMeasuredValues_s.iYawAngle1_d;
	double lMeasLateralAcc_d		= pPrevMeasuredValues_s.iLateralAcceleration_d;
	double lMeasPositionX_d		    = pPrevMeasuredValues_s.iPosition1X_d;
	double lMeasPositionY_d		    = pPrevMeasuredValues_s.iPosition1Y_d;
	double lPrevMeasVehicleSpeed_d  = pPrevMeasuredValues_s.iVehicleSpeed_d;
	double lPrevMeasLateralAcc_d    = pPrevMeasuredValues_s.iLateralAcceleration_d;


	vector<double> lh_v(5);
	vector<double> lxPre_v(5);
	vector<double> lxPro_v(5);
	vector<double> ly_v(5);

	matrix<double> lPPre_m(5, 5);
	matrix<double> lK_m(5, 5);
	matrix<double> lF_m(5, 5);
	matrix<double> lH_m(5, 5);
	matrix<double> lL_m(5, 5);
	matrix<double> lM_m(5, 5);
	matrix<double> lI_m(5, 5);
	
	lPrevMeasVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;

	lh_v(0) = lYawRate_d;
	lh_v(1) = lYawAngle_d;
	lh_v(2) = lLateralAcc_d;
	lh_v(3) = lPositionX_d;
	lh_v(4) = lPositionY_d;

	lxPre_v(0) = lBeta_d;
	lxPre_v(1) = lYawRate_d;
	lxPre_v(2) = lYawAngle_d;
	lxPre_v(3) = lPositionX_d;
	lxPre_v(4) = lPositionY_d;

	ly_v(0) = lMeasYawRate_d;
	ly_v(1) = lMeasYawAngle_d;
	ly_v(2) = lMeasLateralAcc_d;
	ly_v(3) = lMeasPositionX_d;
	ly_v(4) = lMeasPositionY_d;

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
	lM_m(0, 3) = 0;
	lM_m(0, 4) = 0;

	lM_m(1, 0) = 0;
	lM_m(1, 1) = 1;
	lM_m(1, 2) = 0;
	lM_m(1, 3) = 0;
	lM_m(1, 4) = 0;

	lM_m(2, 0) = 0;
	lM_m(2, 1) = 0;
	lM_m(2, 2) = 1;
	lM_m(2, 3) = 0;
	lM_m(2, 4) = 0;

	lM_m(3, 0) = 0;
	lM_m(3, 1) = 0;
	lM_m(3, 2) = 0;
	lM_m(3, 3) = 1;
	lM_m(3, 4) = 0;

	lM_m(4, 0) = 0;
	lM_m(4, 1) = 0;
	lM_m(4, 2) = 0;
	lM_m(4, 3) = 0;
	lM_m(4, 4) = 1;

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
	
	if (lPrevMeasVehicleSpeed_d != 0) {
		lF_m(0, 0) = (1 - pTs_d * ((pVehicleParameters_s.iC1_d + pVehicleParameters_s.iC2_d) / (pVehicleParameters_s.iM_d * lPrevMeasVehicleSpeed_d)));
		lF_m(0, 1) = ((pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d - pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d) / (pVehicleParameters_s.iM_d * (lPrevMeasVehicleSpeed_d * lPrevMeasVehicleSpeed_d)) - 1) * pTs_d;
		lF_m(0, 2) = 0;
		lF_m(0, 3) = 0;
		lF_m(0, 4) = 0;

		lF_m(1, 0) = pTs_d * (-pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d + pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d) / pVehicleParameters_s.iJz_d;
		// TODO: Verify l1^2 and l2^2 is correct
		lF_m(1, 1) = (1 - (pTs_d * (pVehicleParameters_s.iC2_d * (pVehicleParameters_s.iL2_d * pVehicleParameters_s.iL2_d) + pVehicleParameters_s.iC1_d * (pVehicleParameters_s.iL1_d * pVehicleParameters_s.iL1_d)) / (lPrevMeasVehicleSpeed_d * pVehicleParameters_s.iJz_d)));
		lF_m(1, 2) = 0;
		lF_m(1, 3) = 0;
		lF_m(1, 4) = 0;

		lF_m(2, 0) = 0;
		lF_m(2, 1) = pTs_d;
		lF_m(2, 2) = 1;
		lF_m(2, 3) = 0;
		lF_m(2, 4) = 0;

		lF_m(3, 0) = 0;
		lF_m(3, 1) = 0;
		lF_m(3, 2) = sin(lPrevYawAngle_d) * -pTs_d * lPrevMeasVehicleSpeed_d  +
			cos(lPrevYawAngle_d) * lPrevMeasLateralAcc_d * (-(1 / 2) * (pTs_d * pTs_d));
		lF_m(3, 3) = 1;
		lF_m(3, 4) = 0;

		lF_m(4, 0) = 0;
		lF_m(4, 1) = 0;
		lF_m(4, 2) = cos(lPrevYawAngle_d) * pTs_d * lPrevMeasVehicleSpeed_d +
			sin(lPrevYawAngle_d) * lPrevMeasLateralAcc_d * (-(1 / 2) * (pTs_d * pTs_d));
		lF_m(4, 3) = 0;
		lF_m(4, 4) = 1;
	}
	else {
		lF_m(0, 0) = 0;
		lF_m(0, 1) = 0;
		lF_m(0, 2) = 0;
		lF_m(0, 3) = 0;
		lF_m(0, 4) = 0;

		lF_m(1, 0) = 0;
		lF_m(1, 1) = 0;
		lF_m(1, 2) = 0;
		lF_m(1, 3) = 0;
		lF_m(1, 4) = 0;

		lF_m(2, 0) = 0;
		lF_m(2, 1) = 0;
		lF_m(2, 2) = 0;
		lF_m(2, 3) = 0;
		lF_m(2, 4) = 0;

		lF_m(3, 0) = 0;
		lF_m(3, 1) = 0;
		lF_m(3, 2) = 0;
		lF_m(3, 3) = 0;
		lF_m(3, 4) = 0;

		lF_m(4, 0) = 0;
		lF_m(4, 1) = 0;
		lF_m(4, 2) = 0;
		lF_m(4, 3) = 0;
		lF_m(4, 4) = 0;
	}

	if (lPrevMeasVehicleSpeed_d != 0) {
		lH_m(0, 0) = 0;
		lH_m(1, 0) = 0;
		lH_m(2, 0) = -(pVehicleParameters_s.iC1_d + pVehicleParameters_s.iC2_d) / pVehicleParameters_s.iM_d;
		lH_m(3, 0) = 0;
		lH_m(4, 0) = 0;

		lH_m(0, 1) = 1;
		lH_m(1, 1) = 0;
		lH_m(2, 1) = (pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d - pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d) / (pVehicleParameters_s.iM_d * lPrevMeasVehicleSpeed_d);
		lH_m(3, 1) = 0;
		lH_m(4, 1) = 0;

		lH_m(0, 2) = 0;
		lH_m(1, 2) = 1;
		lH_m(2, 2) = 0;
		lH_m(3, 2) = 0;
		lH_m(4, 2) = 0;

		lH_m(0, 3) = 0;
		lH_m(1, 3) = 0;
		lH_m(2, 3) = 0;
		lH_m(3, 3) = 1;
		lH_m(4, 3) = 0;

		lH_m(0, 4) = 0;
		lH_m(1, 4) = 0;
		lH_m(2, 4) = 0;
		lH_m(3, 4) = 0;
		lH_m(4, 4) = 1;
	}
	else {
		lH_m(0, 0) = 0;
		lH_m(1, 0) = 0;
		lH_m(2, 0) = 0;
		lH_m(3, 0) = 0;
		lH_m(4, 0) = 0;

		lH_m(0, 1) = 0;
		lH_m(1, 1) = 0;
		lH_m(2, 1) = 0;
		lH_m(3, 1) = 0;
		lH_m(4, 1) = 0;

		lH_m(0, 2) = 0;
		lH_m(1, 2) = 0;
		lH_m(2, 2) = 0;
		lH_m(3, 2) = 0;
		lH_m(4, 2) = 0;

		lH_m(0, 3) = 0;
		lH_m(1, 3) = 0;
		lH_m(2, 3) = 0;
		lH_m(3, 3) = 0;
		lH_m(4, 3) = 0;

		lH_m(0, 4) = 0;
		lH_m(1, 4) = 0;
		lH_m(2, 4) = 0;
		lH_m(3, 4) = 0;
		lH_m(4, 4) = 0;
	}
	
	matrix<double> ltmpFMultPrevP_m(5, 5);
	matrix<double> ltmpLMultQ_m(5, 5);
	matrix<double> ltmpHMultPPre_m(5, 5);
	matrix<double> ltmpMMultR_m(5, 5);
	matrix<double> ltmpPPreMultHT_m(5, 5);
	matrix<double> ltmpMMultRMultMT_m(5, 5);
	matrix<double> ltmpHMultPPreMultHT_m(5, 5);
	matrix<double> ltmpSumMatrix_m(5, 5);
	matrix<double> ltmpInvMatrix_m(5, 5);

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
	
	pOutModelStates_s.iBeta_d				  = lxPro_v(0);
	pOutModelStates_s.iYawRate_d			  = lxPro_v(1);
	pOutModelStates_s.iYawAngle_d			  = lxPro_v(2);
	pOutModelStates_s.iPositionX_d			  = lxPro_v(3);
	pOutModelStates_s.iPositionY_d			  = lxPro_v(4);
	pOutModelStates_s.iLateralAcceleration_d  = lLateralAcc_d;
	pOutModelStates_s.iLateralVelocity_d	  = lLateralVelocity_d;
	pOutModelStates_s.iLongitudinalVelocity_d = lLongitudinalVelocity_d;

	pOutP_m = prec_prod((lI_m - prec_prod(lK_m, lH_m)), lPPre_m);
}
