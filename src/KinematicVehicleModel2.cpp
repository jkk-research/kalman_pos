#include "KinematicVehicleModel2.h"

#include <math.h>

double kinLongitudinalVelocityCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lLongitudinalSpeed_d = pMeasuredValues_s.vehicleSpeed_d *
        cos(kinBetaCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d) + kinYawAngleCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d));
    
	return lLongitudinalSpeed_d;
}

double kinLateralVelocityCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lLateralSpeed_d = pMeasuredValues_s.vehicleSpeed_d *
        sin(kinBetaCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d) + kinYawAngleCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d));
    
    return lLateralSpeed_d;
}

double kinBetaCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		atan(tan(pMeasuredValues_s.steeringAngle_d) * (pVehicleParameters_s.l2_d / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d)));

	return lReturnValue_d;
}

double kinYawRateCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pMeasuredValues_s.vehicleSpeed_d *
		((tan(pMeasuredValues_s.steeringAngle_d) * cos(kinBetaCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d))) / (pVehicleParameters_s.l2_d + pVehicleParameters_s.l1_d));

	return lReturnValue_d;
}

double kinYawAngleCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (kinYawRateCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d) * pTs_d) + pPrevModelStates_s.yawAngle_d;

	return lReturnValue_d;
}

double kinPositionXCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 
		pPrevModelStates_s.positionX_d
		+
		(kinLongitudinalVelocityCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d) * pTs_d);

	return lReturnValue_d;
}

double kinPositionYCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 
		pPrevModelStates_s.positionY_d
		+
		(kinLateralVelocityCalculation2(pVehicleParameters_s, pMeasuredValues_s, pPrevModelStates_s, pTs_d) * pTs_d);

	return lReturnValue_d;
}

double kinLateralAccCalculation2(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
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
