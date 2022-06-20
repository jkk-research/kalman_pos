#include "DynamicVehicleModel.h"
#include "CombinedVehicleModel.h"

#include <math.h>

double dynLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

double dynLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));
	return lLateralSpeed_d;
}

double dynBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	if (cos(pPrevModelStates_s.beta_d) != 0) {
		lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d / cos(pPrevModelStates_s.beta_d); // vx / cos(beta)
	}
	//lVehicleSpeed = pPrevMeasuredValues_s.vehicleSpeed_d;

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

double dynYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	if (cos(pPrevModelStates_s.beta_d) != 0) {
		lPrevVehicleSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d / cos(pPrevModelStates_s.beta_d); // vx / cos(beta)
	}
	//lVehicleSpeed = pPrevMeasuredValues_s.vehicleSpeed_d;

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

double dynLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	double lVehicleSpeed_d = 0;
	double lReturnValue_d = 0;

	if (cos(pBeta_d) != 0) {
		lVehicleSpeed_d = pMeasuredValues_s.vehicleSpeed_d / cos(pBeta_d); // vx / cos(beta)
	}
	//lVehicleSpeed = pMeasuredValues_s.vehicleSpeed_d;

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

double dynYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.yawRate_d * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

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
