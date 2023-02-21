#include "DynamicVehicleModelEKFwoGNSS.h"
#include "MatrixInverse.hpp"

#include <math.h>

// v_x(k) = v^M(k)
double dynEKFwoGNSSLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

// v_y(k) = v^M(k-1) tg(delta(k))) l_2 / (l_1 + l_2)
double dynEKFwoGNSSLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));
	return lLateralSpeed_d;
}

// beta(k) = 
//		delta(k-1) T_S c_1/(m v(k-1)) + 
//		(1 - T_S (c_1 + c_2)/(m v(k-1))) beta(k-1) + 
//		((c_2 l_2 - c_1 l_1)/(m v^2(k-1)) - 1) T_S dpsi/dt(k-1)
double dynEKFwoGNSSBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	//if (cos(pPrevModelStates_s.beta_d) != 0) {
	//	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d / cos(pPrevModelStates_s.beta_d); // vx / cos(beta)
	//}
	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;

	if ((lPrevVehicleSpeed_d != 0) && (((pVehicleParameters_s.m_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d)) - 1) != 0)) {
		lReturnValue_d =
			(pPrevMeasuredValues_s.steeringAngle_d *
				pTs_d * pVehicleParameters_s.c1_d / (pVehicleParameters_s.m_d * lPrevVehicleSpeed_d))
			+
			((1 - pTs_d * ((pVehicleParameters_s.c1_d + pVehicleParameters_s.c2_d) / (pVehicleParameters_s.m_d * lPrevVehicleSpeed_d))) *
				pPrevModelStates_s.beta_d)

			+
			((((pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d - pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d) / (pVehicleParameters_s.m_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d)) - 1) * pTs_d) *
				pPrevModelStates_s.yawRate_d);
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;
}

// dpsi/dt(k) = 
//		beta(k-1) T_S (c_2 l_2 - c_1 l_1)/J_zz + 
//		dpsi/dt(k-1)(1 - T_S (c_2 l_2^2 + c_1 l_1^2)/(v(k-1) J_zz)) + 
//		delta(k-1) T_S c_1 l_1/J_zz
double dynEKFwoGNSSYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	//if (cos(pPrevModelStates_s.beta_d) != 0) {
	//	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d / cos(pPrevModelStates_s.beta_d); // vx / cos(beta)
	//}
	lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;

	if (lPrevVehicleSpeed_d != 0) {
		lReturnValue_d =
			pPrevModelStates_s.beta_d * pTs_d * (pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d - pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d) / pVehicleParameters_s.jz_d
			+
			pPrevModelStates_s.yawRate_d *
			(1 - pTs_d * (pVehicleParameters_s.c2_d * (pVehicleParameters_s.l2_d * pVehicleParameters_s.l2_d) + pVehicleParameters_s.c1_d * (pVehicleParameters_s.l1_d * pVehicleParameters_s.l1_d)) / (lPrevVehicleSpeed_d * pVehicleParameters_s.jz_d))
			+
			pPrevMeasuredValues_s.steeringAngle_d * pTs_d * pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d / pVehicleParameters_s.jz_d;
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;
}

// a_y(k) = 
//		beta(k) (c_1 + c_2)/m + 
//		(dpsi/dt(k) (c_2 l_2 - c_1 l_1))/(m v(k)) + 
//		delta(k) c_1/m
double dynEKFwoGNSSLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	double lVehicleSpeed_d = 0;
	double lReturnValue_d = 0;

	//if (cos(pBeta_d) != 0) {
	//	lVehicleSpeed_d = pMeasuredValues_s.vehicleSpeed_d / cos(pBeta_d); // vx / cos(beta)
	//}
	lVehicleSpeed_d = pMeasuredValues_s.vehicleSpeed_d;

	if (lVehicleSpeed_d != 0) {
		lReturnValue_d =
			((-pBeta_d) * (pVehicleParameters_s.c1_d + pVehicleParameters_s.c2_d) / pVehicleParameters_s.m_d)
			+
			(pYawRate_d * (pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d - pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d)) /
			(lVehicleSpeed_d * pVehicleParameters_s.m_d)
			+
			pMeasuredValues_s.steeringAngle_d * (pVehicleParameters_s.c1_d / pVehicleParameters_s.m_d);
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;
}

// psi(k) = dpsi/dt(k-1) T_S + psi(k-1)
double dynEKFwoGNSSYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.yawRate_d * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

// x(k) = x(k-1) + cos(psi(k-1)) v(k-1) T_S - sin(psi(k-1)) a_y(k-1) T_S^2/2
double dynEKFwoGNSSPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pPrevModelStates_s.positionX_d
		+
		(cos(pPrevModelStates_s.yawAngle_d) *
			pPrevMeasuredValues_s.vehicleSpeed_d * pTs_d)
		-
		(sin(pPrevModelStates_s.yawAngle_d) *
			pPrevModelStates_s.lateralAcceleration_d * (pTs_d * pTs_d / 2));

	return lReturnValue_d;
}

// y(k) = y(k-1) + v(k-1) T_S sin(psi(k-1)) + cos(psi(k-1)) a_y(k-1) T_S^2
double dynEKFwoGNSSPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pPrevModelStates_s.positionY_d
		+
		(sin(pPrevModelStates_s.yawAngle_d) *
			pPrevMeasuredValues_s.vehicleSpeed_d * pTs_d)
		+
		(cos(pPrevModelStates_s.yawAngle_d) *
			pPrevModelStates_s.lateralAcceleration_d * (pTs_d * pTs_d / 2));

	return lReturnValue_d;
}

// EKF
void dynEKFwoGNSSEstimate(sModelStates &pOutModelStates_s, matrix<double>& pOutP_m, sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d, matrix<double>& pPrevP_m, matrix<double>& pQ_m, matrix<double>& pR_m, bool pUseYawAngle_b) {
	double lYawRate_d = dynEKFwoGNSSYawRateCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lYawAngle_d = dynEKFwoGNSSYawAngleCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPrevYawAngle_d = pPrevModelStates_s.yawAngle_d;
	double lBeta_d = dynEKFwoGNSSBetaCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lLateralAcc_d = dynEKFwoGNSSLateralAccCalculation(pVehicleParameters_s, pMeasuredValues_s, lBeta_d, lYawRate_d);
	double lPositionX_d = dynEKFwoGNSSPositionXCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lPositionY_d = dynEKFwoGNSSPositionYCalculation(pVehicleParameters_s, pPrevMeasuredValues_s, pPrevModelStates_s, pTs_d);
	double lLateralVelocity_d = dynEKFwoGNSSLateralVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	double lLongitudinalVelocity_d = dynEKFwoGNSSLongitudinalVelocityCalculation(pVehicleParameters_s, pMeasuredValues_s, pTs_d);
	double lMesYawRate_d = pPrevMeasuredValues_s.yawRate_d;
	double lMesYawAngle_d = pPrevMeasuredValues_s.yawAngle_d;
	double lMesLateralAcc_d = pPrevMeasuredValues_s.lateralAcceleration_d;
	double lMesPositionX_d = pPrevMeasuredValues_s.positionX_d;
	double lMesPositionY_d = pPrevMeasuredValues_s.positionY_d;
	double lPrevMesVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;
	double lPrevMesLateralAcc_d = pPrevMeasuredValues_s.lateralAcceleration_d;


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

	// TODO: lPrevMeas...
	lPrevMesVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;

	// h = [dpsi/dt; psi; a_y]
	lh_v(0) = lYawRate_d;
	lh_v(1) = lYawAngle_d;
	lh_v(2) = lLateralAcc_d;

	// x_k^- = [beta(k); dpsi/dt(k); psi(k); x(k); y(k)]
	lxPre_v(0) = lBeta_d;
	lxPre_v(1) = lYawRate_d;
	lxPre_v(2) = lYawAngle_d;
	lxPre_v(3) = lPositionX_d;
	lxPre_v(4) = lPositionY_d;

	// y(k) = [dpsi/dt(k); psi(k); a_y(k)]
	ly_v(0) = lMesYawRate_d;
	if (pUseYawAngle_b) {
		// dpsi/dt^M(k)
		ly_v(1) = lMesYawAngle_d;
	} else {
		// psi(k) ~ psi(k-1) + dpsi/dt^M(k) T_S
		//ly_v(1) = pPrevModelStates_s.yawAngle_d + lMesYawRate_d * pTs_d;
		
		// beta(k) = atan(tan(delta) l_2/(l_1 + l_2))
		double lTmpBeta_d = atan(tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)));
		
		// psi(k) = psi(k-1) + T_S v(k) tan(delta(k)) cos(beta(k))/(l_1 + l_2)
		ly_v(1) = pPrevModelStates_s.yawAngle_d + pTs_d*(pMeasuredValues_s.vehicleSpeed_d *((tan(pMeasuredValues_s.steeringAngle_d) * cos(lTmpBeta_d)) / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)));
	}
	ly_v(2) = lMesLateralAcc_d;

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

	// M = I_3
	lM_m(0, 0) = 1;
	lM_m(0, 1) = 0;
	lM_m(0, 2) = 0;

	lM_m(1, 0) = 0;
	lM_m(1, 1) = 1;
	lM_m(1, 2) = 0;

	lM_m(2, 0) = 0;
	lM_m(2, 1) = 0;
	lM_m(2, 2) = 1;
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
	// 1-T_S(c_1+c_2)/(m v(k-1)), ((c_2 l_2-c_1 l_1)/(m v^2(k-1))-1) T_S, 0, 0, 0;
	// T_S(c_2 l_2-c_1 l_1)/J_zz, 1-T_S (c_2 l_2^2+c_1 l_1^2)/(v(k-1) J_zz), 0, 0, 0;
	// 0, T_S, 1, 0, 0;
	// 0, 0, -T_S sin(psi(k-1)) v(k-1) - 1/2 T_S^2 cos(psi(k-1)) a_y(k-1), 1, 0;
	// 0, 0, cos(psi(k-1)) T_S v(k-1) - 1/2 T_S^2 sin(psi(k-1)) a_y(k-1), 0, 1
	// ]
	// x_k = [beta(k); dpsi/dt(k); psi(k); x(k); y(k)]
	if (lPrevMesVehicleSpeed_d != 0) {
		lF_m(0, 0) = (1 - pTs_d * ((pVehicleParameters_s.c1_d + pVehicleParameters_s.c2_d) / (pVehicleParameters_s.m_d * lPrevMesVehicleSpeed_d)));
		lF_m(0, 1) = ((pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d - pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d) / (pVehicleParameters_s.m_d * (lPrevMesVehicleSpeed_d * lPrevMesVehicleSpeed_d)) - 1) * pTs_d;
		lF_m(0, 2) = 0;
		lF_m(0, 3) = 0;
		lF_m(0, 4) = 0;

		lF_m(1, 0) = pTs_d * (-pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d + pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d) / pVehicleParameters_s.jz_d;
		// TODO: Verify l1^2 and l2^2 is correct
		// TODO: Az l_1^2 és l_2^2 helyes.
		// TODO: Lemaradt az elejéről az "1 - "-os rész.
		lF_m(1, 1) = pTs_d * (pVehicleParameters_s.c2_d * (pVehicleParameters_s.l2_d * pVehicleParameters_s.l2_d) + pVehicleParameters_s.c1_d * (pVehicleParameters_s.l1_d * pVehicleParameters_s.l1_d)) / (lPrevMesVehicleSpeed_d * pVehicleParameters_s.jz_d);
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
		lF_m(3, 2) = sin(lPrevYawAngle_d) * -pTs_d * lPrevMesVehicleSpeed_d +
			cos(lPrevYawAngle_d) * lPrevMesLateralAcc_d * (-(1 / 2) * (pTs_d * pTs_d));
		lF_m(3, 3) = 1;
		lF_m(3, 4) = 0;

		lF_m(4, 0) = 0;
		lF_m(4, 1) = 0;
		lF_m(4, 2) = cos(lPrevYawAngle_d) * pTs_d * lPrevMesVehicleSpeed_d +
			sin(lPrevYawAngle_d) * lPrevMesLateralAcc_d * (-(1 / 2) * (pTs_d * pTs_d));
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

	// TODO: A H mátrix sorait a kódban transzponálni kellene! A helyes alak:
	// H_k = dh/dx = [
	// 		0, 1, 0, 0, 0;
	// 		0, 0, 1, 0, 0;
	// 		-(c_1 + c_2)/m, (c_2 l_2 - c_1 l_1)/(m v(k-1)), 0, 0, 0;
	//]
	// h ~= H_k x_k
	// h = [dpsi/dt(k); psi(k); a_y(k)]
	// x_k = [beta(k); dpsi/dt(k); psi(k); x(k); y(k)]
	if (lPrevMesVehicleSpeed_d != 0) {
		lH_m(0, 0) = 0;
		lH_m(1, 0) = 0;
		lH_m(2, 0) = -(pVehicleParameters_s.c1_d + pVehicleParameters_s.c2_d) / pVehicleParameters_s.m_d;

		lH_m(0, 1) = 1;
		lH_m(1, 1) = 0;
		lH_m(2, 1) = (pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d - pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d) / (pVehicleParameters_s.m_d * lPrevMesVehicleSpeed_d);

		lH_m(0, 2) = 0;
		lH_m(1, 2) = 1;
		lH_m(2, 2) = 0;

		lH_m(0, 3) = 0;
		lH_m(1, 3) = 0;
		lH_m(2, 3) = 0;

		lH_m(0, 4) = 0;
		lH_m(1, 4) = 0;
		lH_m(2, 4) = 0;
	}
	else {
		lH_m(0, 0) = 0;
		lH_m(1, 0) = 0;
		lH_m(2, 0) = 0;

		lH_m(0, 1) = 0;
		lH_m(1, 1) = 0;
		lH_m(2, 1) = 0;

		lH_m(0, 2) = 0;
		lH_m(1, 2) = 0;
		lH_m(2, 2) = 0;

		lH_m(0, 3) = 0;
		lH_m(1, 3) = 0;
		lH_m(2, 3) = 0;

		lH_m(0, 4) = 0;
		lH_m(1, 4) = 0;
		lH_m(2, 4) = 0;
	}

	matrix<double> ltmpFMultPrevP_m(5, 5);
	matrix<double> ltmpLMultQ_m(5, 5);
	matrix<double> ltmpHMultPPre_m(3, 5);
	matrix<double> ltmpMMultR_m(3, 3);
	matrix<double> ltmpPPreMultHT_m(5, 3);
	matrix<double> ltmpMMultRMultMT_m(3, 3);
	matrix<double> ltmpHMultPPreMultHT_m(3, 3);
	matrix<double> ltmpSumMatrix_m(3, 3);
	matrix<double> ltmpInvMatrix_m(3, 3);

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

	pOutModelStates_s.beta_d = lxPro_v(0);
	pOutModelStates_s.yawRate_d = lxPro_v(1);
	pOutModelStates_s.yawAngle_d = lxPro_v(2);
	pOutModelStates_s.positionX_d = lxPro_v(3);
	pOutModelStates_s.positionY_d = lxPro_v(4);
	pOutModelStates_s.lateralAcceleration_d = lLateralAcc_d;
	pOutModelStates_s.lateralVelocity_d = lLateralVelocity_d;
	pOutModelStates_s.longitudinalVelocity_d = lLongitudinalVelocity_d;
	
	// P_k^+ = (I - K_k H_k) P_k^-
	pOutP_m = prec_prod((lI_m - prec_prod(lK_m, lH_m)), lPPre_m);
}
