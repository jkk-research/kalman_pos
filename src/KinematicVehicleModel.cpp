#include "KinematicVehicleModel.h"

#include <math.h>

double kinLongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

double kinLateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));
	return lLateralSpeed_d;
}

double kinBetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		atan(tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)));

	return lReturnValue_d;
}

double kinYawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pMeasuredValues_s.vehicleSpeed_d *
		sin(atan(tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)))) /
		pVehicleParameters_s.l2_d;

	return lReturnValue_d;
}

double kinYawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.yawRate_d * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

double kinPositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevLongitudinalSpeed_d = 0;
	double lPrevLateralSpeed_d = 0;

	lPrevLongitudinalSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;
	lPrevLateralSpeed_d = lPrevLongitudinalSpeed_d * tan(pPrevMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));


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

double kinPositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevLongitudinalSpeed_d = 0;
	double lPrevLateralSpeed_d = 0;

	lPrevLongitudinalSpeed_d = pPrevMeasuredValues_s.vehicleSpeed_d;
	lPrevLateralSpeed_d = lPrevLongitudinalSpeed_d * tan(pPrevMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));

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

double kinLateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
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
