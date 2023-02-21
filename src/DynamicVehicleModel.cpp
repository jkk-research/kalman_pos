#include "DynamicVehicleModel.h"
#include "CombinedVehicleModel.h"

#include <math.h>

// v_x^M
double dynLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

// v_y^M
double dynLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));
	return lLateralSpeed_d;
}

// beta(k) = 
//		delta(k-1) T_S c_1/(m v) + 
//		(1 - T_S ((c_1 + c_2)/(m v))) beta(k-1) + 
//		(((c_2 l_2 - c_1 l_1)/(m v^2)) - 1) T_S dpsi/dt(k-1)
double dynBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	
	if (cos(pPrevModelStates_s.beta_d) != 0) {
		// v(k-1) = v_x(k-1) / cos(beta(k-1))
		lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d / cos(pPrevModelStates_s.beta_d);
	}
	
	//lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;

	if ((lPrevVehicleSpeed_d != 0) && (((pVehicleParameters_s.m_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d)) - 1) != 0)) {
		lReturnValue_d =
			(pPrevMeasuredValues_s.steeringAngle_d *
				pTs_d * pVehicleParameters_s.c1_d / (pVehicleParameters_s.m_d * lPrevVehicleSpeed_d))
			+
			((1 - pTs_d * ((pVehicleParameters_s.c1_d + pVehicleParameters_s.c2_d) / (pVehicleParameters_s.m_d * lPrevVehicleSpeed_d))) *
				pPrevModelStates_s.beta_d)

			+
			(((((pVehicleParameters_s.c2_d * pVehicleParameters_s.l2_d - pVehicleParameters_s.c1_d * pVehicleParameters_s.l1_d) / (pVehicleParameters_s.m_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d))) - 1) * pTs_d) *
				pPrevModelStates_s.yawRate_d);
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;	
}

// dpsi/dt(k) = 
//		beta(k-1) T_S (c_2 l_2 - c_1 l_1)/J_zz + 
//		dpsi/dt(k-1) (1 - T_S (c_2 l_2^2 + c_1 l_1^2)/(v(k-1) J_zz)) + 
//		delta(k-1) T_S c_1 l_1/J_zz;
double dynYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	
	if (cos(pPrevModelStates_s.beta_d) != 0) {
		// v(k-1) = v_x^M(k-1) / cos(beta(k-1))
		lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d / cos(pPrevModelStates_s.beta_d);
	}
	
	//lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;

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
//		-beta(k) (c_1 + c_2)/m + 
//		dpsi/dt(k) (c_2 l_2 - c_1 l_1)/(m v(k)) + 
//		delta(k) c_1/m
double dynLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	double lVehicleSpeed_d = 0;
	double lReturnValue_d = 0;
	
	if (cos(pBeta_d) != 0) {
		// v(k) = v_x^M(k) / cos(beta(k))
		lVehicleSpeed_d = pMeasuredValues_s.vehicleSpeed_d / cos(pBeta_d);
	}
	
	//lVehicleSpeed_d = pMeasuredValues_s.vehicleSpeed_d;

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
double dynYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.yawRate_d * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

// x(k) = x(k-1) + v(k-1) T_S cos(psi(k-1)) - a_y(k-1) T_S^2/2 sin(psi(k-1))
double dynPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
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

// y(k) = y(k-1) + v(k-1) T_S sin(psi(k-1)) + a_y(k-1) T_S^2/2 cos(psi(k-1))
double dynPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
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
