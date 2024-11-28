#include "KinematicVehicleModel_woEKFwoExtPos.h"

#include <math.h>

double kin_woEKFwoExtPos_LongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

double kin_woEKFwoExtPos_LateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d));
	return lLateralSpeed_d;
}

double kin_woEKFwoExtPos_BetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		atan(tan(pMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d)));

	return lReturnValue_d;
}

double kin_woEKFwoExtPos_YawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, sModelStates pModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d =
		pMeasuredValues_s.iVehicleSpeed_d *
		sin(atan(tan(pMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d)))) /
		pVehicleParameters_s.iL2_d;

	return lReturnValue_d;
}

double kin_woEKFwoExtPos_YawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.iYawRate_d * pTs_d) + pPrevModelStates_s.iYawAngle_d;

	return lReturnValue_d;
}

double kin_woEKFwoExtPos_PositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevLongitudinalSpeed_d = 0;
	double lPrevLateralSpeed_d = 0;

	lPrevLongitudinalSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;
	lPrevLateralSpeed_d = lPrevLongitudinalSpeed_d * tan(pPrevMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d));


	lReturnValue_d =
		pPrevModelStates_s.iPositionX_d
		+
		(cos(pPrevModelStates_s.iYawAngle_d) * pTs_d *
			lPrevLongitudinalSpeed_d)
		-
		(sin(pPrevModelStates_s.iYawAngle_d) * pTs_d *
			lPrevLateralSpeed_d);

	return lReturnValue_d;
}

double kin_woEKFwoExtPos_PositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevLongitudinalSpeed_d = 0;
	double lPrevLateralSpeed_d = 0;

	lPrevLongitudinalSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;
	lPrevLateralSpeed_d = lPrevLongitudinalSpeed_d * tan(pPrevMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d));

	lReturnValue_d =
		pPrevModelStates_s.iPositionY_d
		+
		(sin(pPrevModelStates_s.iYawAngle_d) * pTs_d *
			lPrevLongitudinalSpeed_d)
		+
		(cos(pPrevModelStates_s.iYawAngle_d) * pTs_d *
			lPrevLateralSpeed_d);

	return lReturnValue_d;
}

double kin_woEKFwoExtPos_LateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	double lVehicleSpeed_d = 0;
	double lReturnValue_d = 0;

	if (cos(pBeta_d) != 0) {
		lVehicleSpeed_d = pMeasuredValues_s.iVehicleSpeed_d / cos(pBeta_d); // vx / cos(beta)
	}
	//lVehicleSpeed = pMeasuredValues_s.vehicleSpeed_d;

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
