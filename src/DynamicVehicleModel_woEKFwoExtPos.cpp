#include "DynamicVehicleModel_woEKFwoExtPos.h"
#include "CombinedVehicleModel.h"

#include <math.h>

double dyn_woEKFwoExtPos_LongitudinalVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;
	return lLongitudinalSpeed_d;
}

double dyn_woEKFwoExtPos_LateralVelocityCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pTs_d) {
	double lLateralSpeed_d = 0;
	double lLongitudinalSpeed_d = 0;

	lLongitudinalSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;
	lLateralSpeed_d = lLongitudinalSpeed_d * tan(pMeasuredValues_s.iSteeringAngle_d) * (pVehicleParameters_s.iL2_d / (pVehicleParameters_s.iL2_d + pVehicleParameters_s.iL1_d));
	return lLateralSpeed_d;
}

double dyn_woEKFwoExtPos_BetaCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	
	if (cos(pPrevModelStates_s.iBeta_d) != 0) {
		lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d / cos(pPrevModelStates_s.iBeta_d); // vx / cos(beta)
	}
	
	//lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;

	if ((lPrevVehicleSpeed_d != 0) && (((pVehicleParameters_s.iM_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d)) - 1) != 0)) {
		lReturnValue_d =
			(pPrevMeasuredValues_s.iSteeringAngle_d *
				pTs_d * pVehicleParameters_s.iC1_d / (pVehicleParameters_s.iM_d * lPrevVehicleSpeed_d))
			+
			((1 - pTs_d * ((pVehicleParameters_s.iC1_d + pVehicleParameters_s.iC2_d) / (pVehicleParameters_s.iM_d * lPrevVehicleSpeed_d))) *
				pPrevModelStates_s.iBeta_d)

			+
			(((((pVehicleParameters_s.iC2_d * pVehicleParameters_s.iL2_d - pVehicleParameters_s.iC1_d * pVehicleParameters_s.iL1_d) / (pVehicleParameters_s.iM_d * (lPrevVehicleSpeed_d * lPrevVehicleSpeed_d))) - 1) * pTs_d) *
				pPrevModelStates_s.iYawRate_d);
	}
	else {
		lReturnValue_d = 0;
	}

	return lReturnValue_d;	
}

double dyn_woEKFwoExtPos_YawRateCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;
	double lPrevVehicleSpeed_d = 0;
	
	if (cos(pPrevModelStates_s.iBeta_d) != 0) {
		lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d / cos(pPrevModelStates_s.iBeta_d); // vx / cos(beta)
	}
	
	//lPrevVehicleSpeed_d = pPrevMeasuredValues_s.iVehicleSpeed_d;

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

double dyn_woEKFwoExtPos_LateralAccCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pMeasuredValues_s, double pBeta_d, double pYawRate_d) {
	double lVehicleSpeed_d = 0;
	double lReturnValue_d = 0;

	
	if (cos(pBeta_d) != 0) {
		lVehicleSpeed_d = pMeasuredValues_s.iVehicleSpeed_d / cos(pBeta_d); // vx / cos(beta)
	}
	

	//lVehicleSpeed_d = pMeasuredValues_s.iVehicleSpeed_d;

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

double dyn_woEKFwoExtPos_YawAngleCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
	double lReturnValue_d = 0;

	lReturnValue_d = (pPrevModelStates_s.iYawRate_d * pTs_d) + pPrevModelStates_s.iYawAngle_d;

	return lReturnValue_d;
}

double dyn_woEKFwoExtPos_PositionXCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
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

double dyn_woEKFwoExtPos_PositionYCalculation(sVehicleParameters pVehicleParameters_s, sMeasuredValues pPrevMeasuredValues_s, sModelStates pPrevModelStates_s, double pTs_d) {
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
