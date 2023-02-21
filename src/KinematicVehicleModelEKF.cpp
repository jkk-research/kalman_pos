#include "KinematicVehicleModelEKF.h"
#include "MatrixInverse.hpp"

#include "ros/ros.h"
#include <math.h>

// v_x(k) = a_x^M(k-1) T_S + v_x(k-1)
double kinEKFLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = pPrevMeasuredValues_s.longitudinalAcceleration_d * pTs_d + pPrevModelStates_s.longitudinalVelocity_d;

	return lReturnValue_d;
}

// v_y(k) = a_y^M(k-1) T_S + v_y(k-1)
double kinEKFLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = pPrevMeasuredValues_s.lateralAcceleration_d * pTs_d + pPrevModelStates_s.lateralVelocity_d;

	return lReturnValue_d;
}

// beta(k-1) = arctg(l_2 / (l_1 + l_2) tg(delta(k-1)))
double kinEKFBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		atan(tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)));

	return lReturnValue_d;
}

// v(k-1) = v_x(k-1)/cos(beta(k-1))
// dpsi/dt(k-1) = v_x(k) / l_2 * tg(beta(k-1))
double kinEKFYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, double pLongitudinalVelocity_d, double pBeta_d) {
	double lReturnValue_d = 0;
	
	lReturnValue_d = pLongitudinalVelocity_d * tan(pBeta_d) / pVehicleParameters_s.l2_d;

	return lReturnValue_d;
}

// v_x^M
double kinEKFMesBasedLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

// v_y^M
double kinEKFMesBasedLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));
	return lLateralSpeed_d;
}

// a_y^M
double kinEKFLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	return pMeasuredValues_s.lateralAcceleration_d;
}

// psi(k) = dpsi/dt(k-1) T_S + psi(k-1)
double kinEKFYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.yawRate_d * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

// x(k) = x(k-1) + v_x(k-1) T_S cos(psi(k-1)) - v_y(k-1) T_S sin(psi(k-1))
double kinEKFPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
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

// y(k) = y(k-1) + v_x(k-1) T_S sin(psi(k-1)) + v_y(k-1) T_S cos(psi(k-1))
double kinEKFPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
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

// EKF
void kinEKFEstimate(sModelStates &pOutModelStates_s, matrix<double>& pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double>& pPrevP_m, matrix<double>& pQ_m, matrix<double>& pR_m) {
	double lYawAngle_d						= kinEKFYawAngleCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPrevYawAngle_d					= pPrevModelStates_s.yawAngle_d;
	double lPrevLongitudinalVelocity_d	    = pPrevModelStates_s.longitudinalVelocity_d;
	double lPrevLateralVelocity_d			= pPrevModelStates_s.lateralVelocity_d;
	double lLongitudinalVelocity_d			= kinEKFLongitudinalVelocityCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lLateralVelocity_d				= kinEKFLateralVelocityCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lBeta_d							= kinEKFBetaCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lYawRate_d						= kinEKFYawRateCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d, lLongitudinalVelocity_d, lBeta_d);
	double lLateralAcc_d					= kinEKFLateralAccCalculation(pVehicleParameters_s, pMeasuredValues_s, lBeta_d, lYawRate_d);
	double lPositionX_d						= kinEKFPositionXCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPositionY_d						= kinEKFPositionYCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lMesBasedLateralVelocity_d		= kinEKFMesBasedLateralVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	double lMesBasedLongitudinalVelocity_d  = kinEKFMesBasedLongitudinalVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	
	/*
	ROS_INFO_STREAM(    "--Model EKF w GNSS Beta: " << lBeta_d <<
						"  YR: " << lYawRate_d << 
						"  YA: " << lYawAngle_d << 
						"  LA: " << lLateralAcc_d << 
						"  X: " << lPositionX_d << 
						"  Y: " << lPositionY_d << 
						"  VX: " << lLongitudinalVelocity_d << 
						"  VY: " << lLateralVelocity_d);
	*/

	double lMesYawRate_d			= pPrevMeasuredValues_s.yawRate_d;
	double lMesYawAngle_d			= pPrevMeasuredValues_s.yawAngle_d;
	double lMesLateralAcc_d			= pPrevMeasuredValues_s.lateralAcceleration_d;
	double lMesPositionX_d			= pPrevMeasuredValues_s.positionX_d;
	double lMesPositionY_d			= pPrevMeasuredValues_s.positionY_d;
	double lPevMesVehicleSpeed_d	= pPrevMeasuredValues_s.vehicleSpeed_d;
	double lPrevMesLateralAcc_d		= pPrevMeasuredValues_s.lateralAcceleration_d;


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

	lPevMesVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;
	
	// h = [v_x; v_y; x; y; dpsi/dt]
	lh_v(0) = lLongitudinalVelocity_d;
	lh_v(1) = lLateralVelocity_d;
	lh_v(2) = lPositionX_d;
	lh_v(3) = lPositionY_d;
	lh_v(4) = lYawRate_d;
	
	// x_k^- = [v_x(k); v_y(k); x(k); y(k); psi(k)]
	lxPre_v(0) = lLongitudinalVelocity_d;
	lxPre_v(1) = lLateralVelocity_d;
	lxPre_v(2) = lPositionX_d;
	lxPre_v(3) = lPositionY_d;
	lxPre_v(4) = lYawAngle_d;
	
	// y(k) = [v_x^ROS(k); v_y^ROS(k); x^GPS(k); y^GPS(k); dpsi/dt(k)]
	ly_v(0) = lMesBasedLongitudinalVelocity_d;
	ly_v(1) = lMesBasedLateralVelocity_d;
	ly_v(2) = lMesPositionX_d;
	ly_v(3) = lMesPositionY_d;
	ly_v(4) = lMesYawRate_d;
	
	// L = I_5
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
	
	// M = I_5
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
	
	// I_5
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
	
	// F_k = [
	// 	1, 0, 0, 0, 0;
	// 	0, 1, 0, 0, 0;
	// 	T_S cos(psi(k-1)), 0, 1, 0, -T_S v_x(k-1) sin(psi(k-1));
	// 	0, T_S sin(psi(k-1)), 0, 1, T_S v_y(k-1) cos(psi(k-1));
	// 	0, 0, 0, 0, 1
	// ]
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
	lF_m(3, 4) = pTs_d * (lPrevLateralVelocity_d) * cos(lPrevYawAngle_d);

	lF_m(4, 0) = 0;
	lF_m(4, 1) = 0;
	lF_m(4, 2) = 0;
	lF_m(4, 3) = 0;
	lF_m(4, 4) = 1;
	
	// H_k = dh/dx = [
	// 		1, 0, 0, 0, 0;
	// 		0, 1, 0, 0, 0;
	// 		0, 0, 1, 0, 0;
	// 		0, 0, 0, 1, 0;
	// 		d/dv_x(dpsi/dt), 0, 0, 0, 0;
	//]
	// h ~= H_k x_k
	// h = [v_x; v_y; x; y; dpsi/dt]
	// x_k = [v_x; v_y; x; y; psi]
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

	lH_m(2, 0) = 0;
	lH_m(2, 1) = 0;
	lH_m(2, 2) = 1;
	lH_m(2, 3) = 0;
	lH_m(2, 4) = 0;

	lH_m(3, 0) = 0;
	lH_m(3, 1) = 0;
	lH_m(3, 2) = 0;
	lH_m(3, 3) = 1;
	lH_m(3, 4) = 0;

	// TODO: tan or sin (it have to be checked)?
	// dpsi/dt = 
	//		v/l_2 sin(beta) = 
	//		v_x/cos(beta)/l_2 sin(beta) = 
	//		v_x tg(beta)/l_2
	lH_m(4, 0) = pVehicleParameters_s.l2_d * tan(lBeta_d);
	lH_m(4, 1) = 0;
	lH_m(4, 2) = 0;
	lH_m(4, 3) = 0;
	lH_m(4, 4) = 0;
	
	// K_k
	matrix<double> ltmpFMultPrevP_m(5, 5);
	matrix<double> ltmpLMultQ_m(5, 5);
	matrix<double> ltmpHMultPPre_m(5, 5);
	matrix<double> ltmpMMultR_m(5, 5);
	matrix<double> ltmpPPreMultHT_m(5, 5);
	matrix<double> ltmpMMultRMultMT_m(5, 5);
	matrix<double> ltmpHMultPPreMultHT_m(5, 5);
	matrix<double> ltmpSumMatrix_m(5, 5);
	matrix<double> ltmpInvMatrix_m(5, 5);
	
	// P_k^- = F P_k-1^+ F^T + L Q L^T
	ltmpFMultPrevP_m = prec_prod(lF_m, pPrevP_m);
	ltmpLMultQ_m = prec_prod(lL_m, pQ_m);
	ltmpMMultR_m = prec_prod(lM_m, pR_m);
	lPPre_m = prec_prod(ltmpFMultPrevP_m, trans(lF_m)) + prec_prod(ltmpLMultQ_m, trans(lL_m));
	
	ltmpHMultPPre_m = prec_prod(lH_m, lPPre_m);
	
	// P_k^- H_k^T
	ltmpPPreMultHT_m = prec_prod(lPPre_m, trans(lH_m));
	ltmpMMultRMultMT_m = prec_prod(ltmpMMultR_m, trans(lM_m));
	ltmpHMultPPreMultHT_m = prec_prod(ltmpHMultPPre_m, trans(lH_m));
	ltmpSumMatrix_m = ltmpHMultPPreMultHT_m + ltmpMMultRMultMT_m;
	InvertMatrix(ltmpSumMatrix_m, ltmpInvMatrix_m);
	
	// K_k = P_k^- H_k^T (H_k P_k^- H_k^T + M_k R_k M_k^T)
	lK_m = prec_prod(ltmpPPreMultHT_m, ltmpInvMatrix_m);
	
	// \hat{x}_k^+ = \hat{x}_k^- + K_k (y_k - h_k(\hat{x}_k^-, u_k))
	lxPro_v = lxPre_v + prec_prod(lK_m, (ly_v - lh_v));
	
	pOutModelStates_s.beta_d				  = lBeta_d;
	pOutModelStates_s.yawRate_d				  = lYawRate_d;
	pOutModelStates_s.yawAngle_d			  = lxPro_v(4);
	pOutModelStates_s.positionX_d			  = lxPro_v(2);
	pOutModelStates_s.positionY_d			  = lxPro_v(3);
	pOutModelStates_s.lateralAcceleration_d   = lLateralAcc_d;
	pOutModelStates_s.longitudinalVelocity_d  = lxPro_v(0);
	pOutModelStates_s.lateralVelocity_d       = lxPro_v(1);
	
	// P_k^+ = (I - K_k H_k) P_k^-
	pOutP_m = prec_prod((lI_m - prec_prod(lK_m, lH_m)), lPPre_m);
}
