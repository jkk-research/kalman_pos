#include "CombinedVehicleModel.h"
#include "rclcpp/rclcpp.hpp"
#include <sstream>


cCombinedVehicleModel::cCombinedVehicleModel() {
    sVehicleParameters lVehicleParameters_s;
    lVehicleParameters_s.iC1_d   = 3000;//4000;
    lVehicleParameters_s.iC2_d   = 800;//2400; // The ratio is very important!!!!!
    lVehicleParameters_s.iM_d    = 180;
    lVehicleParameters_s.iJz_d   = 270;
    lVehicleParameters_s.iL1_d   = 1.3 - 0.976;
    lVehicleParameters_s.iL2_d   = 0.976;
    lVehicleParameters_s.iSwr_d  = 1;

    initEKFMatrices();
    initVehicleParameters(lVehicleParameters_s);
}

cCombinedVehicleModel::cCombinedVehicleModel(sVehicleParameters &pVehicleParameters_s) {
        initEKFMatrices();
        initVehicleParameters(pVehicleParameters_s);
}

cCombinedVehicleModel::~cCombinedVehicleModel() {

}

void cCombinedVehicleModel::initEKFMatrices(void) {
    iPDyn_wEKFwExtPos1_m = matrix<double>(5, 5);
    iQDyn_wEKFwExtPos1_m = matrix<double>(5, 5);
    iRDyn_wEKFwExtPos1_m = matrix<double>(5, 5);
    iPKin_wEKFwExtPos1_m = matrix<double>(5, 5);
    iQKin_wEKFwExtPos1_m = matrix<double>(5, 5);
    iRKin_wEKFwExtPos1_m = matrix<double>(5, 5);

    iPDyn_wEKFwExtPos2_m = matrix<double>(5, 5);
    iQDyn_wEKFwExtPos2_m = matrix<double>(5, 5);
    iRDyn_wEKFwExtPos2_m = matrix<double>(5, 5);
    iPKin_wEKFwExtPos2_m = matrix<double>(5, 5);
    iQKin_wEKFwExtPos2_m = matrix<double>(5, 5);
    iRKin_wEKFwExtPos2_m = matrix<double>(5, 5);

    iPDyn_wEKFwExtPos1Pos2_m = matrix<double>(5, 5);
    iQDyn_wEKFwExtPos1Pos2_m = matrix<double>(5, 5);
    iRDyn_wEKFwExtPos1Pos2_m = matrix<double>(7, 7);
    iPKin_wEKFwExtPos1Pos2_m = matrix<double>(5, 5);
    iQKin_wEKFwExtPos1Pos2_m = matrix<double>(5, 5);
    iRKin_wEKFwExtPos1Pos2_m = matrix<double>(7, 7);

    iPDyn_wEKFwoExtPos_m = matrix<double>(5, 5);
    iQDyn_wEKFwoExtPos_m = matrix<double>(5, 5);
    iRDyn_wEKFwoExtPos_m = matrix<double>(3, 3);
    iPKin_wEKFwoExtPos_m = matrix<double>(5, 5);
    iQKin_wEKFwoExtPos_m = matrix<double>(5, 5);
    iRKin_wEKFwoExtPos_m = matrix<double>(3, 3);

    iPrevPDyn_wEKFwExtPos1_m = matrix<double>(5, 5);
    iPrevPKin_wEKFwExtPos1_m = matrix<double>(5, 5);

    iPrevPDyn_wEKFwExtPos2_m = matrix<double>(5, 5);
    iPrevPKin_wEKFwExtPos2_m = matrix<double>(5, 5);

    iPrevPDyn_wEKFwExtPos1Pos2_m = matrix<double>(5, 5);
    iPrevPKin_wEKFwExtPos1Pos2_m = matrix<double>(5, 5);

    iPrevPDyn_wEKFwoExtPos_m = matrix<double>(5, 5);
    iPrevPKin_wEKFwoExtPos_m = matrix<double>(5, 5);

    iPDyn_wEKFwExtPos1_m(0, 0) = 1;
    iPDyn_wEKFwExtPos1_m(0, 1) = 0;
    iPDyn_wEKFwExtPos1_m(0, 2) = 0;
    iPDyn_wEKFwExtPos1_m(0, 3) = 0;
    iPDyn_wEKFwExtPos1_m(0, 4) = 0;
    iPDyn_wEKFwExtPos1_m(1, 0) = 0;
    iPDyn_wEKFwExtPos1_m(1, 1) = 1;
    iPDyn_wEKFwExtPos1_m(1, 2) = 0;
    iPDyn_wEKFwExtPos1_m(1, 3) = 0;
    iPDyn_wEKFwExtPos1_m(1, 4) = 0;
    iPDyn_wEKFwExtPos1_m(2, 0) = 0;
    iPDyn_wEKFwExtPos1_m(2, 1) = 0;
    iPDyn_wEKFwExtPos1_m(2, 2) = 1;
    iPDyn_wEKFwExtPos1_m(2, 3) = 0;
    iPDyn_wEKFwExtPos1_m(2, 4) = 0;
    iPDyn_wEKFwExtPos1_m(3, 0) = 0;
    iPDyn_wEKFwExtPos1_m(3, 1) = 0;
    iPDyn_wEKFwExtPos1_m(3, 2) = 0;
    iPDyn_wEKFwExtPos1_m(3, 3) = 1;
    iPDyn_wEKFwExtPos1_m(3, 4) = 0;
    iPDyn_wEKFwExtPos1_m(4, 0) = 0;
    iPDyn_wEKFwExtPos1_m(4, 1) = 0;
    iPDyn_wEKFwExtPos1_m(4, 2) = 0;
    iPDyn_wEKFwExtPos1_m(4, 3) = 0;
    iPDyn_wEKFwExtPos1_m(4, 4) = 1;

    iQDyn_wEKFwExtPos1_m(0, 0) = 0.0145;
    iQDyn_wEKFwExtPos1_m(0, 1) = 0;
    iQDyn_wEKFwExtPos1_m(0, 2) = 0;
    iQDyn_wEKFwExtPos1_m(0, 3) = 0;
    iQDyn_wEKFwExtPos1_m(0, 4) = 0;
    iQDyn_wEKFwExtPos1_m(1, 0) = 0;
    iQDyn_wEKFwExtPos1_m(1, 1) = 1.4821;
    iQDyn_wEKFwExtPos1_m(1, 2) = 0;
    iQDyn_wEKFwExtPos1_m(1, 3) = 0;
    iQDyn_wEKFwExtPos1_m(1, 4) = 0;
    iQDyn_wEKFwExtPos1_m(2, 0) = 0;
    iQDyn_wEKFwExtPos1_m(2, 1) = 0;
    iQDyn_wEKFwExtPos1_m(2, 2) = 1.929482792522855e-04;
    iQDyn_wEKFwExtPos1_m(2, 3) = 0;
    iQDyn_wEKFwExtPos1_m(2, 4) = 0;
    iQDyn_wEKFwExtPos1_m(3, 0) = 0;
    iQDyn_wEKFwExtPos1_m(3, 1) = 0;
    iQDyn_wEKFwExtPos1_m(3, 2) = 0;
    iQDyn_wEKFwExtPos1_m(3, 3) = 2.002;
    iQDyn_wEKFwExtPos1_m(3, 4) = 0;
    iQDyn_wEKFwExtPos1_m(4, 0) = 0;
    iQDyn_wEKFwExtPos1_m(4, 1) = 0;
    iQDyn_wEKFwExtPos1_m(4, 2) = 0;
    iQDyn_wEKFwExtPos1_m(4, 3) = 0;
    iQDyn_wEKFwExtPos1_m(4, 4) = 1.9060;

    iRDyn_wEKFwExtPos1_m(0, 0) = 3.729484658649690e-04;
    iRDyn_wEKFwExtPos1_m(0, 1) = 0;
    iRDyn_wEKFwExtPos1_m(0, 2) = 0;
    iRDyn_wEKFwExtPos1_m(0, 3) = 0;
    iRDyn_wEKFwExtPos1_m(0, 4) = 0;
    iRDyn_wEKFwExtPos1_m(1, 0) = 0;
    iRDyn_wEKFwExtPos1_m(1, 1) = 1.750794749765446;
    iRDyn_wEKFwExtPos1_m(1, 2) = 0;
    iRDyn_wEKFwExtPos1_m(1, 3) = 0;
    iRDyn_wEKFwExtPos1_m(1, 4) = 0;
    iRDyn_wEKFwExtPos1_m(2, 0) = 0;
    iRDyn_wEKFwExtPos1_m(2, 1) = 0;
    iRDyn_wEKFwExtPos1_m(2, 2) = 1.543640292721019;
    iRDyn_wEKFwExtPos1_m(2, 3) = 0;
    iRDyn_wEKFwExtPos1_m(2, 4) = 0;
    iRDyn_wEKFwExtPos1_m(3, 0) = 0;
    iRDyn_wEKFwExtPos1_m(3, 1) = 0;
    iRDyn_wEKFwExtPos1_m(3, 2) = 0;
    iRDyn_wEKFwExtPos1_m(3, 3) = 10;
    iRDyn_wEKFwExtPos1_m(3, 4) = 0;
    iRDyn_wEKFwExtPos1_m(4, 0) = 0;
    iRDyn_wEKFwExtPos1_m(4, 1) = 0;
    iRDyn_wEKFwExtPos1_m(4, 2) = 0;
    iRDyn_wEKFwExtPos1_m(4, 3) = 0;
    iRDyn_wEKFwExtPos1_m(4, 4) = 10;

    iPKin_wEKFwExtPos1_m(0, 0) = 1;
    iPKin_wEKFwExtPos1_m(0, 1) = 0;
    iPKin_wEKFwExtPos1_m(0, 2) = 0;
    iPKin_wEKFwExtPos1_m(0, 3) = 0;
    iPKin_wEKFwExtPos1_m(0, 4) = 0;
    iPKin_wEKFwExtPos1_m(1, 0) = 0;
    iPKin_wEKFwExtPos1_m(1, 1) = 1;
    iPKin_wEKFwExtPos1_m(1, 2) = 0;
    iPKin_wEKFwExtPos1_m(1, 3) = 0;
    iPKin_wEKFwExtPos1_m(1, 4) = 0;
    iPKin_wEKFwExtPos1_m(2, 0) = 0;
    iPKin_wEKFwExtPos1_m(2, 1) = 0;
    iPKin_wEKFwExtPos1_m(2, 2) = 1;
    iPKin_wEKFwExtPos1_m(2, 3) = 0;
    iPKin_wEKFwExtPos1_m(2, 4) = 0;
    iPKin_wEKFwExtPos1_m(3, 0) = 0;
    iPKin_wEKFwExtPos1_m(3, 1) = 0;
    iPKin_wEKFwExtPos1_m(3, 2) = 0;
    iPKin_wEKFwExtPos1_m(3, 3) = 1;
    iPKin_wEKFwExtPos1_m(3, 4) = 0;
    iPKin_wEKFwExtPos1_m(4, 0) = 0;
    iPKin_wEKFwExtPos1_m(4, 1) = 0;
    iPKin_wEKFwExtPos1_m(4, 2) = 0;
    iPKin_wEKFwExtPos1_m(4, 3) = 0;
    iPKin_wEKFwExtPos1_m(4, 4) = 1;

    iQKin_wEKFwExtPos1_m(0, 0) = 1.750800000000000;
    iQKin_wEKFwExtPos1_m(0, 1) = 0;
    iQKin_wEKFwExtPos1_m(0, 2) = 0;
    iQKin_wEKFwExtPos1_m(0, 3) = 0;
    iQKin_wEKFwExtPos1_m(0, 4) = 0;
    iQKin_wEKFwExtPos1_m(1, 0) = 0;
    iQKin_wEKFwExtPos1_m(1, 1) = 1.750800000000000;
    iQKin_wEKFwExtPos1_m(1, 2) = 0;
    iQKin_wEKFwExtPos1_m(1, 3) = 0;
    iQKin_wEKFwExtPos1_m(1, 4) = 0;
    iQKin_wEKFwExtPos1_m(2, 0) = 0;
    iQKin_wEKFwExtPos1_m(2, 1) = 0;
    iQKin_wEKFwExtPos1_m(2, 2) = 2;
    iQKin_wEKFwExtPos1_m(2, 3) = 0;
    iQKin_wEKFwExtPos1_m(2, 4) = 0;
    iQKin_wEKFwExtPos1_m(3, 0) = 0;
    iQKin_wEKFwExtPos1_m(3, 1) = 0;
    iQKin_wEKFwExtPos1_m(3, 2) = 0;
    iQKin_wEKFwExtPos1_m(3, 3) = 2;
    iQKin_wEKFwExtPos1_m(3, 4) = 0;
    iQKin_wEKFwExtPos1_m(4, 0) = 0;
    iQKin_wEKFwExtPos1_m(4, 1) = 0;
    iQKin_wEKFwExtPos1_m(4, 2) = 0;
    iQKin_wEKFwExtPos1_m(4, 3) = 0;
    iQKin_wEKFwExtPos1_m(4, 4) = 0.02;

    iRKin_wEKFwExtPos1_m(0, 0) = 0.5;
    iRKin_wEKFwExtPos1_m(0, 1) = 0;
    iRKin_wEKFwExtPos1_m(0, 2) = 0;
    iRKin_wEKFwExtPos1_m(0, 3) = 0;
    iRKin_wEKFwExtPos1_m(0, 4) = 0;
    iRKin_wEKFwExtPos1_m(1, 0) = 0;
    iRKin_wEKFwExtPos1_m(1, 1) = 0.5;
    iRKin_wEKFwExtPos1_m(1, 2) = 0;
    iRKin_wEKFwExtPos1_m(1, 3) = 0;
    iRKin_wEKFwExtPos1_m(1, 4) = 0;
    iRKin_wEKFwExtPos1_m(2, 0) = 0;
    iRKin_wEKFwExtPos1_m(2, 1) = 0;
    iRKin_wEKFwExtPos1_m(2, 2) = 10;
    iRKin_wEKFwExtPos1_m(2, 3) = 0;
    iRKin_wEKFwExtPos1_m(2, 4) = 0;
    iRKin_wEKFwExtPos1_m(3, 0) = 0;
    iRKin_wEKFwExtPos1_m(3, 1) = 0;
    iRKin_wEKFwExtPos1_m(3, 2) = 0;
    iRKin_wEKFwExtPos1_m(3, 3) = 10;
    iRKin_wEKFwExtPos1_m(3, 4) = 0;
    iRKin_wEKFwExtPos1_m(4, 0) = 0;
    iRKin_wEKFwExtPos1_m(4, 1) = 0;
    iRKin_wEKFwExtPos1_m(4, 2) = 0;
    iRKin_wEKFwExtPos1_m(4, 3) = 0;
    iRKin_wEKFwExtPos1_m(4, 4) = 1.75;

    iPDyn_wEKFwExtPos2_m(0, 0) = 1;
    iPDyn_wEKFwExtPos2_m(0, 1) = 0;
    iPDyn_wEKFwExtPos2_m(0, 2) = 0;
    iPDyn_wEKFwExtPos2_m(0, 3) = 0;
    iPDyn_wEKFwExtPos2_m(0, 4) = 0;
    iPDyn_wEKFwExtPos2_m(1, 0) = 0;
    iPDyn_wEKFwExtPos2_m(1, 1) = 1;
    iPDyn_wEKFwExtPos2_m(1, 2) = 0;
    iPDyn_wEKFwExtPos2_m(1, 3) = 0;
    iPDyn_wEKFwExtPos2_m(1, 4) = 0;
    iPDyn_wEKFwExtPos2_m(2, 0) = 0;
    iPDyn_wEKFwExtPos2_m(2, 1) = 0;
    iPDyn_wEKFwExtPos2_m(2, 2) = 1;
    iPDyn_wEKFwExtPos2_m(2, 3) = 0;
    iPDyn_wEKFwExtPos2_m(2, 4) = 0;
    iPDyn_wEKFwExtPos2_m(3, 0) = 0;
    iPDyn_wEKFwExtPos2_m(3, 1) = 0;
    iPDyn_wEKFwExtPos2_m(3, 2) = 0;
    iPDyn_wEKFwExtPos2_m(3, 3) = 1;
    iPDyn_wEKFwExtPos2_m(3, 4) = 0;
    iPDyn_wEKFwExtPos2_m(4, 0) = 0;
    iPDyn_wEKFwExtPos2_m(4, 1) = 0;
    iPDyn_wEKFwExtPos2_m(4, 2) = 0;
    iPDyn_wEKFwExtPos2_m(4, 3) = 0;
    iPDyn_wEKFwExtPos2_m(4, 4) = 1;

    iQDyn_wEKFwExtPos2_m(0, 0) = 0.0145;
    iQDyn_wEKFwExtPos2_m(0, 1) = 0;
    iQDyn_wEKFwExtPos2_m(0, 2) = 0;
    iQDyn_wEKFwExtPos2_m(0, 3) = 0;
    iQDyn_wEKFwExtPos2_m(0, 4) = 0;
    iQDyn_wEKFwExtPos2_m(1, 0) = 0;
    iQDyn_wEKFwExtPos2_m(1, 1) = 1.4821;
    iQDyn_wEKFwExtPos2_m(1, 2) = 0;
    iQDyn_wEKFwExtPos2_m(1, 3) = 0;
    iQDyn_wEKFwExtPos2_m(1, 4) = 0;
    iQDyn_wEKFwExtPos2_m(2, 0) = 0;
    iQDyn_wEKFwExtPos2_m(2, 1) = 0;
    iQDyn_wEKFwExtPos2_m(2, 2) = 1.929482792522855e-04;
    iQDyn_wEKFwExtPos2_m(2, 3) = 0;
    iQDyn_wEKFwExtPos2_m(2, 4) = 0;
    iQDyn_wEKFwExtPos2_m(3, 0) = 0;
    iQDyn_wEKFwExtPos2_m(3, 1) = 0;
    iQDyn_wEKFwExtPos2_m(3, 2) = 0;
    iQDyn_wEKFwExtPos2_m(3, 3) = 2.002;
    iQDyn_wEKFwExtPos2_m(3, 4) = 0;
    iQDyn_wEKFwExtPos2_m(4, 0) = 0;
    iQDyn_wEKFwExtPos2_m(4, 1) = 0;
    iQDyn_wEKFwExtPos2_m(4, 2) = 0;
    iQDyn_wEKFwExtPos2_m(4, 3) = 0;
    iQDyn_wEKFwExtPos2_m(4, 4) = 1.9060;

    iRDyn_wEKFwExtPos2_m(0, 0) = 3.729484658649690e-04;
    iRDyn_wEKFwExtPos2_m(0, 1) = 0;
    iRDyn_wEKFwExtPos2_m(0, 2) = 0;
    iRDyn_wEKFwExtPos2_m(0, 3) = 0;
    iRDyn_wEKFwExtPos2_m(0, 4) = 0;
    iRDyn_wEKFwExtPos2_m(1, 0) = 0;
    iRDyn_wEKFwExtPos2_m(1, 1) = 1.750794749765446;
    iRDyn_wEKFwExtPos2_m(1, 2) = 0;
    iRDyn_wEKFwExtPos2_m(1, 3) = 0;
    iRDyn_wEKFwExtPos2_m(1, 4) = 0;
    iRDyn_wEKFwExtPos2_m(2, 0) = 0;
    iRDyn_wEKFwExtPos2_m(2, 1) = 0;
    iRDyn_wEKFwExtPos2_m(2, 2) = 1.543640292721019;
    iRDyn_wEKFwExtPos2_m(2, 3) = 0;
    iRDyn_wEKFwExtPos2_m(2, 4) = 0;
    iRDyn_wEKFwExtPos2_m(3, 0) = 0;
    iRDyn_wEKFwExtPos2_m(3, 1) = 0;
    iRDyn_wEKFwExtPos2_m(3, 2) = 0;
    iRDyn_wEKFwExtPos2_m(3, 3) = 10;
    iRDyn_wEKFwExtPos2_m(3, 4) = 0;
    iRDyn_wEKFwExtPos2_m(4, 0) = 0;
    iRDyn_wEKFwExtPos2_m(4, 1) = 0;
    iRDyn_wEKFwExtPos2_m(4, 2) = 0;
    iRDyn_wEKFwExtPos2_m(4, 3) = 0;
    iRDyn_wEKFwExtPos2_m(4, 4) = 10;

    iPKin_wEKFwExtPos2_m(0, 0) = 1;
    iPKin_wEKFwExtPos2_m(0, 1) = 0;
    iPKin_wEKFwExtPos2_m(0, 2) = 0;
    iPKin_wEKFwExtPos2_m(0, 3) = 0;
    iPKin_wEKFwExtPos2_m(0, 4) = 0;
    iPKin_wEKFwExtPos2_m(1, 0) = 0;
    iPKin_wEKFwExtPos2_m(1, 1) = 1;
    iPKin_wEKFwExtPos2_m(1, 2) = 0;
    iPKin_wEKFwExtPos2_m(1, 3) = 0;
    iPKin_wEKFwExtPos2_m(1, 4) = 0;
    iPKin_wEKFwExtPos2_m(2, 0) = 0;
    iPKin_wEKFwExtPos2_m(2, 1) = 0;
    iPKin_wEKFwExtPos2_m(2, 2) = 1;
    iPKin_wEKFwExtPos2_m(2, 3) = 0;
    iPKin_wEKFwExtPos2_m(2, 4) = 0;
    iPKin_wEKFwExtPos2_m(3, 0) = 0;
    iPKin_wEKFwExtPos2_m(3, 1) = 0;
    iPKin_wEKFwExtPos2_m(3, 2) = 0;
    iPKin_wEKFwExtPos2_m(3, 3) = 1;
    iPKin_wEKFwExtPos2_m(3, 4) = 0;
    iPKin_wEKFwExtPos2_m(4, 0) = 0;
    iPKin_wEKFwExtPos2_m(4, 1) = 0;
    iPKin_wEKFwExtPos2_m(4, 2) = 0;
    iPKin_wEKFwExtPos2_m(4, 3) = 0;
    iPKin_wEKFwExtPos2_m(4, 4) = 1;

    iQKin_wEKFwExtPos2_m(0, 0) = 1.750800000000000;
    iQKin_wEKFwExtPos2_m(0, 1) = 0;
    iQKin_wEKFwExtPos2_m(0, 2) = 0;
    iQKin_wEKFwExtPos2_m(0, 3) = 0;
    iQKin_wEKFwExtPos2_m(0, 4) = 0;
    iQKin_wEKFwExtPos2_m(1, 0) = 0;
    iQKin_wEKFwExtPos2_m(1, 1) = 1.750800000000000;
    iQKin_wEKFwExtPos2_m(1, 2) = 0;
    iQKin_wEKFwExtPos2_m(1, 3) = 0;
    iQKin_wEKFwExtPos2_m(1, 4) = 0;
    iQKin_wEKFwExtPos2_m(2, 0) = 0;
    iQKin_wEKFwExtPos2_m(2, 1) = 0;
    iQKin_wEKFwExtPos2_m(2, 2) = 2;
    iQKin_wEKFwExtPos2_m(2, 3) = 0;
    iQKin_wEKFwExtPos2_m(2, 4) = 0;
    iQKin_wEKFwExtPos2_m(3, 0) = 0;
    iQKin_wEKFwExtPos2_m(3, 1) = 0;
    iQKin_wEKFwExtPos2_m(3, 2) = 0;
    iQKin_wEKFwExtPos2_m(3, 3) = 2;
    iQKin_wEKFwExtPos2_m(3, 4) = 0;
    iQKin_wEKFwExtPos2_m(4, 0) = 0;
    iQKin_wEKFwExtPos2_m(4, 1) = 0;
    iQKin_wEKFwExtPos2_m(4, 2) = 0;
    iQKin_wEKFwExtPos2_m(4, 3) = 0;
    iQKin_wEKFwExtPos2_m(4, 4) = 0.02;

    iRKin_wEKFwExtPos2_m(0, 0) = 0.5;
    iRKin_wEKFwExtPos2_m(0, 1) = 0;
    iRKin_wEKFwExtPos2_m(0, 2) = 0;
    iRKin_wEKFwExtPos2_m(0, 3) = 0;
    iRKin_wEKFwExtPos2_m(0, 4) = 0;
    iRKin_wEKFwExtPos2_m(1, 0) = 0;
    iRKin_wEKFwExtPos2_m(1, 1) = 0.5;
    iRKin_wEKFwExtPos2_m(1, 2) = 0;
    iRKin_wEKFwExtPos2_m(1, 3) = 0;
    iRKin_wEKFwExtPos2_m(1, 4) = 0;
    iRKin_wEKFwExtPos2_m(2, 0) = 0;
    iRKin_wEKFwExtPos2_m(2, 1) = 0;
    iRKin_wEKFwExtPos2_m(2, 2) = 10;
    iRKin_wEKFwExtPos2_m(2, 3) = 0;
    iRKin_wEKFwExtPos2_m(2, 4) = 0;
    iRKin_wEKFwExtPos2_m(3, 0) = 0;
    iRKin_wEKFwExtPos2_m(3, 1) = 0;
    iRKin_wEKFwExtPos2_m(3, 2) = 0;
    iRKin_wEKFwExtPos2_m(3, 3) = 10;
    iRKin_wEKFwExtPos2_m(3, 4) = 0;
    iRKin_wEKFwExtPos2_m(4, 0) = 0;
    iRKin_wEKFwExtPos2_m(4, 1) = 0;
    iRKin_wEKFwExtPos2_m(4, 2) = 0;
    iRKin_wEKFwExtPos2_m(4, 3) = 0;
    iRKin_wEKFwExtPos2_m(4, 4) = 1.75;

    iPDyn_wEKFwExtPos1Pos2_m(0, 0) = 1;
    iPDyn_wEKFwExtPos1Pos2_m(0, 1) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(0, 2) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(0, 3) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(0, 4) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(1, 0) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(1, 1) = 1;
    iPDyn_wEKFwExtPos1Pos2_m(1, 2) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(1, 3) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(1, 4) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(2, 0) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(2, 1) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(2, 2) = 1;
    iPDyn_wEKFwExtPos1Pos2_m(2, 3) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(2, 4) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(3, 0) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(3, 1) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(3, 2) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(3, 3) = 1;
    iPDyn_wEKFwExtPos1Pos2_m(3, 4) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(4, 0) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(4, 1) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(4, 2) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(4, 3) = 0;
    iPDyn_wEKFwExtPos1Pos2_m(4, 4) = 1;

    iQDyn_wEKFwExtPos1Pos2_m(0, 0) = 0.0145;
    iQDyn_wEKFwExtPos1Pos2_m(0, 1) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(0, 2) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(0, 3) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(0, 4) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(1, 0) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(1, 1) = 1.4821;
    iQDyn_wEKFwExtPos1Pos2_m(1, 2) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(1, 3) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(1, 4) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(2, 0) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(2, 1) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(2, 2) = 1.929482792522855e-04;
    iQDyn_wEKFwExtPos1Pos2_m(2, 3) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(2, 4) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(3, 0) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(3, 1) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(3, 2) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(3, 3) = 2.002;
    iQDyn_wEKFwExtPos1Pos2_m(3, 4) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(4, 0) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(4, 1) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(4, 2) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(4, 3) = 0;
    iQDyn_wEKFwExtPos1Pos2_m(4, 4) = 1.9060;

    iRDyn_wEKFwExtPos1Pos2_m(0, 0) = 3.729484658649690e-04;
    iRDyn_wEKFwExtPos1Pos2_m(0, 1) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(0, 2) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(0, 3) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(0, 4) = 0;
	iRDyn_wEKFwExtPos1Pos2_m(0, 5) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(0, 6) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(1, 0) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(1, 1) = 1.750794749765446;
    iRDyn_wEKFwExtPos1Pos2_m(1, 2) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(1, 3) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(1, 4) = 0;
	iRDyn_wEKFwExtPos1Pos2_m(1, 5) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(1, 6) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(2, 0) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(2, 1) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(2, 2) = 1.543640292721019;
    iRDyn_wEKFwExtPos1Pos2_m(2, 3) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(2, 4) = 0;
	iRDyn_wEKFwExtPos1Pos2_m(2, 5) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(2, 6) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(3, 0) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(3, 1) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(3, 2) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(3, 3) = 10;
    iRDyn_wEKFwExtPos1Pos2_m(3, 4) = 0;
	iRDyn_wEKFwExtPos1Pos2_m(3, 5) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(3, 6) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(4, 0) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(4, 1) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(4, 2) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(4, 3) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(4, 4) = 10;
    iRDyn_wEKFwExtPos1Pos2_m(4, 5) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(4, 6) = 0;	
	iRDyn_wEKFwExtPos1Pos2_m(5, 0) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(5, 1) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(5, 2) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(5, 3) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(5, 4) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(5, 5) = 10;
    iRDyn_wEKFwExtPos1Pos2_m(5, 6) = 0;
	iRDyn_wEKFwExtPos1Pos2_m(6, 0) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(6, 1) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(6, 2) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(6, 3) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(6, 4) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(6, 5) = 0;
    iRDyn_wEKFwExtPos1Pos2_m(6, 6) = 10;
	
    iPKin_wEKFwExtPos1Pos2_m(0, 0) = 1;
    iPKin_wEKFwExtPos1Pos2_m(0, 1) = 0;
    iPKin_wEKFwExtPos1Pos2_m(0, 2) = 0;
    iPKin_wEKFwExtPos1Pos2_m(0, 3) = 0;
    iPKin_wEKFwExtPos1Pos2_m(0, 4) = 0;
    iPKin_wEKFwExtPos1Pos2_m(1, 0) = 0;
    iPKin_wEKFwExtPos1Pos2_m(1, 1) = 1;
    iPKin_wEKFwExtPos1Pos2_m(1, 2) = 0;
    iPKin_wEKFwExtPos1Pos2_m(1, 3) = 0;
    iPKin_wEKFwExtPos1Pos2_m(1, 4) = 0;
    iPKin_wEKFwExtPos1Pos2_m(2, 0) = 0;
    iPKin_wEKFwExtPos1Pos2_m(2, 1) = 0;
    iPKin_wEKFwExtPos1Pos2_m(2, 2) = 1;
    iPKin_wEKFwExtPos1Pos2_m(2, 3) = 0;
    iPKin_wEKFwExtPos1Pos2_m(2, 4) = 0;
    iPKin_wEKFwExtPos1Pos2_m(3, 0) = 0;
    iPKin_wEKFwExtPos1Pos2_m(3, 1) = 0;
    iPKin_wEKFwExtPos1Pos2_m(3, 2) = 0;
    iPKin_wEKFwExtPos1Pos2_m(3, 3) = 1;
    iPKin_wEKFwExtPos1Pos2_m(3, 4) = 0;
    iPKin_wEKFwExtPos1Pos2_m(4, 0) = 0;
    iPKin_wEKFwExtPos1Pos2_m(4, 1) = 0;
    iPKin_wEKFwExtPos1Pos2_m(4, 2) = 0;
    iPKin_wEKFwExtPos1Pos2_m(4, 3) = 0;
    iPKin_wEKFwExtPos1Pos2_m(4, 4) = 1;

    iQKin_wEKFwExtPos1Pos2_m(0, 0) = 1.750800000000000;
    iQKin_wEKFwExtPos1Pos2_m(0, 1) = 0;
    iQKin_wEKFwExtPos1Pos2_m(0, 2) = 0;
    iQKin_wEKFwExtPos1Pos2_m(0, 3) = 0;
    iQKin_wEKFwExtPos1Pos2_m(0, 4) = 0;
    iQKin_wEKFwExtPos1Pos2_m(1, 0) = 0;
    iQKin_wEKFwExtPos1Pos2_m(1, 1) = 1.750800000000000;
    iQKin_wEKFwExtPos1Pos2_m(1, 2) = 0;
    iQKin_wEKFwExtPos1Pos2_m(1, 3) = 0;
    iQKin_wEKFwExtPos1Pos2_m(1, 4) = 0;
    iQKin_wEKFwExtPos1Pos2_m(2, 0) = 0;
    iQKin_wEKFwExtPos1Pos2_m(2, 1) = 0;
    iQKin_wEKFwExtPos1Pos2_m(2, 2) = 2;
    iQKin_wEKFwExtPos1Pos2_m(2, 3) = 0;
    iQKin_wEKFwExtPos1Pos2_m(2, 4) = 0;
    iQKin_wEKFwExtPos1Pos2_m(3, 0) = 0;
    iQKin_wEKFwExtPos1Pos2_m(3, 1) = 0;
    iQKin_wEKFwExtPos1Pos2_m(3, 2) = 0;
    iQKin_wEKFwExtPos1Pos2_m(3, 3) = 2;
    iQKin_wEKFwExtPos1Pos2_m(3, 4) = 0;
    iQKin_wEKFwExtPos1Pos2_m(4, 0) = 0;
    iQKin_wEKFwExtPos1Pos2_m(4, 1) = 0;
    iQKin_wEKFwExtPos1Pos2_m(4, 2) = 0;
    iQKin_wEKFwExtPos1Pos2_m(4, 3) = 0;
    iQKin_wEKFwExtPos1Pos2_m(4, 4) = 0.02;

    iRKin_wEKFwExtPos1Pos2_m(0, 0) = 0.5;
    iRKin_wEKFwExtPos1Pos2_m(0, 1) = 0;
    iRKin_wEKFwExtPos1Pos2_m(0, 2) = 0;
    iRKin_wEKFwExtPos1Pos2_m(0, 3) = 0;
    iRKin_wEKFwExtPos1Pos2_m(0, 4) = 0;
	iRKin_wEKFwExtPos1Pos2_m(0, 5) = 0;
	iRKin_wEKFwExtPos1Pos2_m(0, 6) = 0;
    iRKin_wEKFwExtPos1Pos2_m(1, 0) = 0;
    iRKin_wEKFwExtPos1Pos2_m(1, 1) = 0.5;
    iRKin_wEKFwExtPos1Pos2_m(1, 2) = 0;
    iRKin_wEKFwExtPos1Pos2_m(1, 3) = 0;
    iRKin_wEKFwExtPos1Pos2_m(1, 4) = 0;
	iRKin_wEKFwExtPos1Pos2_m(1, 5) = 0;
    iRKin_wEKFwExtPos1Pos2_m(1, 6) = 0;
    iRKin_wEKFwExtPos1Pos2_m(2, 0) = 0;
    iRKin_wEKFwExtPos1Pos2_m(2, 1) = 0;
    iRKin_wEKFwExtPos1Pos2_m(2, 2) = 10;
    iRKin_wEKFwExtPos1Pos2_m(2, 3) = 0;
    iRKin_wEKFwExtPos1Pos2_m(2, 4) = 0;
	iRKin_wEKFwExtPos1Pos2_m(2, 5) = 0;
    iRKin_wEKFwExtPos1Pos2_m(2, 6) = 0;
    iRKin_wEKFwExtPos1Pos2_m(3, 0) = 0;
    iRKin_wEKFwExtPos1Pos2_m(3, 1) = 0;
    iRKin_wEKFwExtPos1Pos2_m(3, 2) = 0;
    iRKin_wEKFwExtPos1Pos2_m(3, 3) = 10;
    iRKin_wEKFwExtPos1Pos2_m(3, 4) = 0;
	iRKin_wEKFwExtPos1Pos2_m(3, 5) = 0;
	iRKin_wEKFwExtPos1Pos2_m(3, 6) = 0;
    iRKin_wEKFwExtPos1Pos2_m(4, 0) = 0;
    iRKin_wEKFwExtPos1Pos2_m(4, 1) = 0;
    iRKin_wEKFwExtPos1Pos2_m(4, 2) = 0;
    iRKin_wEKFwExtPos1Pos2_m(4, 3) = 0;
	iRKin_wEKFwExtPos1Pos2_m(4, 4) = 10;
    iRKin_wEKFwExtPos1Pos2_m(4, 5) = 0;
	iRKin_wEKFwExtPos1Pos2_m(4, 6) = 0;
    iRKin_wEKFwExtPos1Pos2_m(5, 0) = 0;
    iRKin_wEKFwExtPos1Pos2_m(5, 1) = 0;
    iRKin_wEKFwExtPos1Pos2_m(5, 2) = 0;
    iRKin_wEKFwExtPos1Pos2_m(5, 3) = 0;
	iRKin_wEKFwExtPos1Pos2_m(5, 4) = 0;
    iRKin_wEKFwExtPos1Pos2_m(5, 5) = 10;
	iRKin_wEKFwExtPos1Pos2_m(5, 6) = 0;	
    iRKin_wEKFwExtPos1Pos2_m(6, 0) = 0;
    iRKin_wEKFwExtPos1Pos2_m(6, 1) = 0;
    iRKin_wEKFwExtPos1Pos2_m(6, 2) = 0;
    iRKin_wEKFwExtPos1Pos2_m(6, 3) = 0;
	iRKin_wEKFwExtPos1Pos2_m(6, 4) = 0;
    iRKin_wEKFwExtPos1Pos2_m(6, 5) = 0;
    iRKin_wEKFwExtPos1Pos2_m(6, 6) = 1.75;

    iPDyn_wEKFwoExtPos_m(0, 0) = 1;
    iPDyn_wEKFwoExtPos_m(0, 1) = 0;
    iPDyn_wEKFwoExtPos_m(0, 2) = 0;
    iPDyn_wEKFwoExtPos_m(0, 3) = 0;
    iPDyn_wEKFwoExtPos_m(0, 4) = 0;
    iPDyn_wEKFwoExtPos_m(1, 0) = 0;
    iPDyn_wEKFwoExtPos_m(1, 1) = 1;
    iPDyn_wEKFwoExtPos_m(1, 2) = 0;
    iPDyn_wEKFwoExtPos_m(1, 3) = 0;
    iPDyn_wEKFwoExtPos_m(1, 4) = 0;
    iPDyn_wEKFwoExtPos_m(2, 0) = 0;
    iPDyn_wEKFwoExtPos_m(2, 1) = 0;
    iPDyn_wEKFwoExtPos_m(2, 2) = 1;
    iPDyn_wEKFwoExtPos_m(2, 3) = 0;
    iPDyn_wEKFwoExtPos_m(2, 4) = 0;
    iPDyn_wEKFwoExtPos_m(3, 0) = 0;
    iPDyn_wEKFwoExtPos_m(3, 1) = 0;
    iPDyn_wEKFwoExtPos_m(3, 2) = 0;
    iPDyn_wEKFwoExtPos_m(3, 3) = 1;
    iPDyn_wEKFwoExtPos_m(3, 4) = 0;
    iPDyn_wEKFwoExtPos_m(4, 0) = 0;
    iPDyn_wEKFwoExtPos_m(4, 1) = 0;
    iPDyn_wEKFwoExtPos_m(4, 2) = 0;
    iPDyn_wEKFwoExtPos_m(4, 3) = 0;
    iPDyn_wEKFwoExtPos_m(4, 4) = 1;

    iQDyn_wEKFwoExtPos_m(0, 0) = 0.0145;
    iQDyn_wEKFwoExtPos_m(0, 1) = 0;
    iQDyn_wEKFwoExtPos_m(0, 2) = 0;
    iQDyn_wEKFwoExtPos_m(0, 3) = 0;
    iQDyn_wEKFwoExtPos_m(0, 4) = 0;
    iQDyn_wEKFwoExtPos_m(1, 0) = 0;
    iQDyn_wEKFwoExtPos_m(1, 1) = 1.4821; // yaw rate
    iQDyn_wEKFwoExtPos_m(1, 2) = 0;
    iQDyn_wEKFwoExtPos_m(1, 3) = 0;
    iQDyn_wEKFwoExtPos_m(1, 4) = 0;
    iQDyn_wEKFwoExtPos_m(2, 0) = 0;
    iQDyn_wEKFwoExtPos_m(2, 1) = 0;
    iQDyn_wEKFwoExtPos_m(2, 2) = 1.929482792522855e-04; // yaw angle
    iQDyn_wEKFwoExtPos_m(2, 3) = 0;
    iQDyn_wEKFwoExtPos_m(2, 4) = 0;
    iQDyn_wEKFwoExtPos_m(3, 0) = 0;
    iQDyn_wEKFwoExtPos_m(3, 1) = 0;
    iQDyn_wEKFwoExtPos_m(3, 2) = 0;
    iQDyn_wEKFwoExtPos_m(3, 3) = 2.002;
    iQDyn_wEKFwoExtPos_m(3, 4) = 0;
    iQDyn_wEKFwoExtPos_m(4, 0) = 0;
    iQDyn_wEKFwoExtPos_m(4, 1) = 0;
    iQDyn_wEKFwoExtPos_m(4, 2) = 0;
    iQDyn_wEKFwoExtPos_m(4, 3) = 0;
    iQDyn_wEKFwoExtPos_m(4, 4) = 1.9060;

    iRDyn_wEKFwoExtPos_m(0, 0) = 3.729484658649690e-04; // yaw rate
    iRDyn_wEKFwoExtPos_m(0, 1) = 0;
    iRDyn_wEKFwoExtPos_m(0, 2) = 0;
    iRDyn_wEKFwoExtPos_m(1, 0) = 0;
    iRDyn_wEKFwoExtPos_m(1, 1) = 1.750794749765446; // yaw angle
    iRDyn_wEKFwoExtPos_m(1, 2) = 0;
    iRDyn_wEKFwoExtPos_m(2, 0) = 0;
    iRDyn_wEKFwoExtPos_m(2, 1) = 0;
    iRDyn_wEKFwoExtPos_m(2, 2) = 1.543640292721019;

    iPKin_wEKFwoExtPos_m(0, 0) = 1;
    iPKin_wEKFwoExtPos_m(0, 1) = 0;
    iPKin_wEKFwoExtPos_m(0, 2) = 0;
    iPKin_wEKFwoExtPos_m(0, 3) = 0;
    iPKin_wEKFwoExtPos_m(0, 4) = 0;
    iPKin_wEKFwoExtPos_m(1, 0) = 0;
    iPKin_wEKFwoExtPos_m(1, 1) = 1;
    iPKin_wEKFwoExtPos_m(1, 2) = 0;
    iPKin_wEKFwoExtPos_m(1, 3) = 0;
    iPKin_wEKFwoExtPos_m(1, 4) = 0;
    iPKin_wEKFwoExtPos_m(2, 0) = 0;
    iPKin_wEKFwoExtPos_m(2, 1) = 0;
    iPKin_wEKFwoExtPos_m(2, 2) = 1;
    iPKin_wEKFwoExtPos_m(2, 3) = 0;
    iPKin_wEKFwoExtPos_m(2, 4) = 0;
    iPKin_wEKFwoExtPos_m(3, 0) = 0;
    iPKin_wEKFwoExtPos_m(3, 1) = 0;
    iPKin_wEKFwoExtPos_m(3, 2) = 0;
    iPKin_wEKFwoExtPos_m(3, 3) = 1;
    iPKin_wEKFwoExtPos_m(3, 4) = 0;
    iPKin_wEKFwoExtPos_m(4, 0) = 0;
    iPKin_wEKFwoExtPos_m(4, 1) = 0;
    iPKin_wEKFwoExtPos_m(4, 2) = 0;
    iPKin_wEKFwoExtPos_m(4, 3) = 0;
    iPKin_wEKFwoExtPos_m(4, 4) = 1;

    iQKin_wEKFwoExtPos_m(0, 0) = 1.750800000000000;
    iQKin_wEKFwoExtPos_m(0, 1) = 0;
    iQKin_wEKFwoExtPos_m(0, 2) = 0;
    iQKin_wEKFwoExtPos_m(0, 3) = 0;
    iQKin_wEKFwoExtPos_m(0, 4) = 0;
    iQKin_wEKFwoExtPos_m(1, 0) = 0;
    iQKin_wEKFwoExtPos_m(1, 1) = 1.750800000000000;
    iQKin_wEKFwoExtPos_m(1, 2) = 0;
    iQKin_wEKFwoExtPos_m(1, 3) = 0;
    iQKin_wEKFwoExtPos_m(1, 4) = 0;
    iQKin_wEKFwoExtPos_m(2, 0) = 0;
    iQKin_wEKFwoExtPos_m(2, 1) = 0;
    iQKin_wEKFwoExtPos_m(2, 2) = 2;
    iQKin_wEKFwoExtPos_m(2, 3) = 0;
    iQKin_wEKFwoExtPos_m(2, 4) = 0;
    iQKin_wEKFwoExtPos_m(3, 0) = 0;
    iQKin_wEKFwoExtPos_m(3, 1) = 0;
    iQKin_wEKFwoExtPos_m(3, 2) = 0;
    iQKin_wEKFwoExtPos_m(3, 3) = 2;
    iQKin_wEKFwoExtPos_m(3, 4) = 0;
    iQKin_wEKFwoExtPos_m(4, 0) = 0;
    iQKin_wEKFwoExtPos_m(4, 1) = 0;
    iQKin_wEKFwoExtPos_m(4, 2) = 0;
    iQKin_wEKFwoExtPos_m(4, 3) = 0;
    iQKin_wEKFwoExtPos_m(4, 4) = 0.02;

    iRKin_wEKFwoExtPos_m(0, 0) = 0.5;
    iRKin_wEKFwoExtPos_m(0, 1) = 0;
    iRKin_wEKFwoExtPos_m(0, 2) = 0;
    iRKin_wEKFwoExtPos_m(1, 0) = 0;
    iRKin_wEKFwoExtPos_m(1, 1) = 0.5;
    iRKin_wEKFwoExtPos_m(1, 2) = 0;
    iRKin_wEKFwoExtPos_m(2, 0) = 0;
    iRKin_wEKFwoExtPos_m(2, 1) = 0;
    iRKin_wEKFwoExtPos_m(2, 2) = 1.75;

    setPrevEKFMatrices();
}

void cCombinedVehicleModel::setPrevEKFMatrices(void) {
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            iPrevPDyn_wEKFwExtPos1_m(i, j)  = iPDyn_wEKFwExtPos1_m(i, j);
            iPrevPKin_wEKFwExtPos1_m(i, j)  = iPKin_wEKFwExtPos1_m(i, j);
            iPrevPDyn_wEKFwExtPos2_m(i, j)  = iPDyn_wEKFwExtPos2_m(i, j);
            iPrevPKin_wEKFwExtPos2_m(i, j)  = iPKin_wEKFwExtPos2_m(i, j);
            iPrevPDyn_wEKFwExtPos1Pos2_m(i, j)  = iPDyn_wEKFwExtPos1Pos2_m(i, j);
            iPrevPKin_wEKFwExtPos1Pos2_m(i, j)  = iPKin_wEKFwExtPos1Pos2_m(i, j);
            iPrevPDyn_wEKFwoExtPos_m(i, j)  = iPDyn_wEKFwoExtPos_m(i, j);
            iPrevPKin_wEKFwoExtPos_m(i, j)  = iPKin_wEKFwoExtPos_m(i, j);
        }
    }
}

void cCombinedVehicleModel::initVehicleParameters( sVehicleParameters &pVehicleParameters_s) {
    iVehicleParameters_s.iC1_d   = pVehicleParameters_s.iC1_d;
    iVehicleParameters_s.iC2_d   = pVehicleParameters_s.iC2_d;
    iVehicleParameters_s.iM_d    = pVehicleParameters_s.iM_d;
    iVehicleParameters_s.iJz_d   = pVehicleParameters_s.iJz_d;
    iVehicleParameters_s.iL1_d   = pVehicleParameters_s.iL1_d;
    iVehicleParameters_s.iL2_d   = pVehicleParameters_s.iL2_d;
    iVehicleParameters_s.iSwr_d  = pVehicleParameters_s.iSwr_d;
}

void cCombinedVehicleModel::setMeasuredValuesVehicleState(double pSteeringAngle_d, double pVehicleSpeed_d) {
    iMeasuredValues_s.iSteeringAngle_d   = pSteeringAngle_d / iVehicleParameters_s.iSwr_d;
    iMeasuredValues_s.iVehicleSpeed_d    = pVehicleSpeed_d;
}

void cCombinedVehicleModel::setMeasuredValuesGNSS(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d) {
    iMeasuredValues_s.iPosition1X_d = pPositionX_d;
    iMeasuredValues_s.iPosition1Y_d = pPositionY_d;
    iMeasuredValues_s.iPosition1Z_d = pPositionZ_d;
    iMeasuredValues_s.iYawAngle1_d  = pYawAngle_d;
}

void cCombinedVehicleModel::setMeasuredValuesSLAM(double pPositionX_d, double pPositionY_d, double pPositionZ_d, double pYawAngle_d) {
    iMeasuredValues_s.iPosition2X_d = pPositionX_d;
    iMeasuredValues_s.iPosition2Y_d = pPositionY_d;
    iMeasuredValues_s.iPosition2Z_d = pPositionZ_d;
    iMeasuredValues_s.iYawAngle2_d  = pYawAngle_d;
}

void cCombinedVehicleModel::setMeasuredValuesIMU(double pLongitudinalAcceleration_d, double pLateralAcceleration_d, double pVerticalAcceleration_d, double pRollRate_d, double pPitchRate_d, double pYawRate_d) {
    iMeasuredValues_s.iLongitudinalAcceleration_d = pLongitudinalAcceleration_d;
    iMeasuredValues_s.iLateralAcceleration_d      = pLateralAcceleration_d;
    iMeasuredValues_s.iVerticalAcceleration_d     = pVerticalAcceleration_d;
    iMeasuredValues_s.iRollRate_d                 = pRollRate_d;
    iMeasuredValues_s.iPitchRate_d                = pPitchRate_d;
    iMeasuredValues_s.iYawRate_d                  = pYawRate_d;
}

void cCombinedVehicleModel::setPrevMeasuredValues() {
    iPrevMeasuredValues_s.iSteeringAngle_d            = iMeasuredValues_s.iSteeringAngle_d;
    iPrevMeasuredValues_s.iVehicleSpeed_d             = iMeasuredValues_s.iVehicleSpeed_d;
    iPrevMeasuredValues_s.iPosition1X_d               = iMeasuredValues_s.iPosition1X_d;
    iPrevMeasuredValues_s.iPosition1Y_d               = iMeasuredValues_s.iPosition1Y_d;
    iPrevMeasuredValues_s.iPosition1Z_d               = iMeasuredValues_s.iPosition1Z_d;
    iPrevMeasuredValues_s.iPosition2X_d               = iMeasuredValues_s.iPosition2X_d;
    iPrevMeasuredValues_s.iPosition2Y_d               = iMeasuredValues_s.iPosition2Y_d;
    iPrevMeasuredValues_s.iPosition2Z_d               = iMeasuredValues_s.iPosition2Z_d;
    iPrevMeasuredValues_s.iYawAngle1_d                = iMeasuredValues_s.iYawAngle1_d;
    iPrevMeasuredValues_s.iYawAngle2_d                = iMeasuredValues_s.iYawAngle2_d;
    iPrevMeasuredValues_s.iLongitudinalAcceleration_d = iMeasuredValues_s.iLongitudinalAcceleration_d;
    iPrevMeasuredValues_s.iLateralAcceleration_d      = iMeasuredValues_s.iLateralAcceleration_d;
    iPrevMeasuredValues_s.iVerticalAcceleration_d     = iMeasuredValues_s.iVerticalAcceleration_d;
    iPrevMeasuredValues_s.iRollRate_d                 = iMeasuredValues_s.iRollRate_d;
    iPrevMeasuredValues_s.iPitchRate_d                = iMeasuredValues_s.iPitchRate_d;
    iPrevMeasuredValues_s.iYawRate_d                  = iMeasuredValues_s.iYawRate_d;
}

void cCombinedVehicleModel::setModelStates(double pBeta_d, double pYawRate_d, double pYawAngle_d, double pLateralAcceleration_d, double pPositionX_d, double pPositionY_d, double pLongitudinalVelocity_d, double pLateralVelocity_d) {
    iModelStates_s.iBeta_d                 = pBeta_d;
    iModelStates_s.iLateralAcceleration_d  = pLateralAcceleration_d;
    iModelStates_s.iPositionX_d            = pPositionX_d;
    iModelStates_s.iPositionY_d            = pPositionY_d;
    iModelStates_s.iYawAngle_d             = pYawAngle_d;
    iModelStates_s.iYawRate_d              = pYawRate_d;
    iModelStates_s.iLateralVelocity_d      = pLateralVelocity_d;
    iModelStates_s.iLongitudinalVelocity_d = pLongitudinalVelocity_d;
}

void cCombinedVehicleModel::setPrevModelStates(void) {
    iPrevModelStates_s.iBeta_d                 = iModelStates_s.iBeta_d;
    iPrevModelStates_s.iLateralAcceleration_d  = iModelStates_s.iLateralAcceleration_d;
    iPrevModelStates_s.iPositionX_d            = iModelStates_s.iPositionX_d;
    iPrevModelStates_s.iPositionY_d            = iModelStates_s.iPositionY_d;
    iPrevModelStates_s.iYawAngle_d             = iModelStates_s.iYawAngle_d;
    iPrevModelStates_s.iYawRate_d              = iModelStates_s.iYawRate_d;
    iPrevModelStates_s.iLateralVelocity_d      = iModelStates_s.iLateralVelocity_d;
    iPrevModelStates_s.iLongitudinalVelocity_d = iModelStates_s.iLongitudinalVelocity_d;
}

void cCombinedVehicleModel::iterateModel(double pTs_d, bool pUseRawModel_b, bool pGNSSAvailable_b, bool pSLAMAvailable_b, double pKinSpeedLimit_d) {
    double lBeta_d                = 0;
    double lLateralAcceleration_d = 0;
    double lPositionX_d           = 0;
    double lPositionY_d           = 0;
    double lYawAngle_d            = 0;
    double lYawRate_d             = 0;
    double lLateralSpeed_d        = 0;
    double lLongitudinalSpeed_d   = 0;

    if (iPrevMeasuredValues_s.iVehicleSpeed_d < pKinSpeedLimit_d) {
        // Kinematic models
        if (pUseRawModel_b) {
            // Model only
            setPrevModelStates();
            setPrevEKFMatrices();

            lBeta_d                 = kin_woEKFwoExtPos_BetaCalculation(iVehicleParameters_s, iMeasuredValues_s, iModelStates_s, pTs_d);
            //lYawRate_d              = iMeasuredValues_s.iYawRate_d;//kinYawRateCalculation(iVehicleParameters_s, iMeasuredValues_s, iModelStates_s, pTs_d);
            lYawRate_d              = kin_woEKFwoExtPos_YawRateCalculation(iVehicleParameters_s, iMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawAngle_d             = kin_woEKFwoExtPos_YawAngleCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralAcceleration_d  = iMeasuredValues_s.iLateralAcceleration_d;//kinLateralAccCalculation(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
            lPositionX_d            = kin_woEKFwoExtPos_PositionXCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lPositionY_d            = kin_woEKFwoExtPos_PositionYCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLongitudinalSpeed_d    = kin_woEKFwoExtPos_LongitudinalVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
            lLateralSpeed_d         = kin_woEKFwoExtPos_LateralVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
            
            setModelStates(lBeta_d, lYawRate_d, lYawAngle_d, lLateralAcceleration_d, lPositionX_d, lPositionY_d, lLongitudinalSpeed_d, lLateralSpeed_d);
            setPrevMeasuredValues();
        } else {
            if ((pGNSSAvailable_b) && (pSLAMAvailable_b)) {
                // Kinematic model with EKF + Ext. Pos. 1 and Pos. 2
                setPrevModelStates();
                setPrevEKFMatrices();
                kin_wEKFwExtPos1Pos2_Estimate(
                    iModelStates_s,
                    iPKin_wEKFwExtPos1Pos2_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s, 
                    pTs_d, 
                    iPrevPKin_wEKFwExtPos1Pos2_m,
                    iQKin_wEKFwExtPos1Pos2_m, 
                    iRKin_wEKFwExtPos1Pos2_m);
                setPrevMeasuredValues();
            } else if (pGNSSAvailable_b) {
                // Kinematic model with EKF + Ext. Pos. 1
                setPrevModelStates();
                setPrevEKFMatrices();
                kin_wEKFwExtPos1_Estimate(
                    iModelStates_s,
                    iPKin_wEKFwExtPos1_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s, 
                    pTs_d, 
                    iPrevPKin_wEKFwExtPos1_m,
                    iQKin_wEKFwExtPos1_m, 
                    iRKin_wEKFwExtPos1_m);
                setPrevMeasuredValues();
            } else if (pSLAMAvailable_b) {
                // Kinematic model with EKF + Ext. Pos. 2
                setPrevModelStates();
                setPrevEKFMatrices();
                kin_wEKFwExtPos2_Estimate(
                    iModelStates_s,
                    iPKin_wEKFwExtPos2_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s, 
                    pTs_d, 
                    iPrevPKin_wEKFwExtPos2_m,
                    iQKin_wEKFwExtPos2_m, 
                    iRKin_wEKFwExtPos2_m);
                setPrevMeasuredValues();
            } else {
                // Kinematic model with EKF without Ext. Pos.
                setPrevModelStates();
                setPrevEKFMatrices();
                kin_wEKFwoExtPos_Estimate(
                    iModelStates_s,
                    iPKin_wEKFwoExtPos_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s, 
                    pTs_d, 
                    iPrevPKin_wEKFwoExtPos_m,
                    iQKin_wEKFwoExtPos_m, 
                    iRKin_wEKFwoExtPos_m);
                setPrevMeasuredValues();
            }
        }
    } else {
        // Dynamic models
        if (pUseRawModel_b) {
            // Model only
            setPrevModelStates();
            setPrevEKFMatrices();

            lBeta_d                 = dyn_woEKFwoExtPos_BetaCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawRate_d              = dyn_woEKFwoExtPos_YawRateCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lYawAngle_d             = dyn_woEKFwoExtPos_YawAngleCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLateralAcceleration_d  = dyn_woEKFwoExtPos_LateralAccCalculation(iVehicleParameters_s, iMeasuredValues_s, lBeta_d, lYawRate_d);
            lPositionX_d            = dyn_woEKFwoExtPos_PositionXCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lPositionY_d            = dyn_woEKFwoExtPos_PositionYCalculation(iVehicleParameters_s, iPrevMeasuredValues_s, iPrevModelStates_s, pTs_d);
            lLongitudinalSpeed_d    = dyn_woEKFwoExtPos_LongitudinalVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
            lLateralSpeed_d         = dyn_woEKFwoExtPos_LateralVelocityCalculation(iVehicleParameters_s, iMeasuredValues_s, pTs_d);
         
            setModelStates(lBeta_d, lYawRate_d, lYawAngle_d, lLateralAcceleration_d, lPositionX_d, lPositionY_d, lLongitudinalSpeed_d, lLateralSpeed_d);
            setPrevMeasuredValues();
        } else {
            if ((pGNSSAvailable_b) && (pSLAMAvailable_b)){
                // Dynamic model with EKF + Ext. Pos. 1 and Pos. 2
                setPrevModelStates();
                setPrevEKFMatrices();
                dyn_wEKFwExtPos1Pos2_Estimate(
                    iModelStates_s,
                    iPDyn_wEKFwExtPos1Pos2_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s,
                    pTs_d,
                    iPrevPDyn_wEKFwExtPos1Pos2_m,
                    iQDyn_wEKFwExtPos1Pos2_m,
                    iRDyn_wEKFwExtPos1Pos2_m);
                setPrevMeasuredValues();
            } else if (pGNSSAvailable_b) {
                // Dynamic model with EKF + Ext. Pos. 1
                setPrevModelStates();
                setPrevEKFMatrices();
                dyn_wEKFwExtPos1_Estimate(
                    iModelStates_s,
                    iPDyn_wEKFwExtPos1_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s,
                    pTs_d,
                    iPrevPDyn_wEKFwExtPos1_m,
                    iQDyn_wEKFwExtPos1_m,
                    iRDyn_wEKFwExtPos1_m);
                setPrevMeasuredValues();
            } else if (pSLAMAvailable_b) {
                // Dynamic model with EKF + Ext. Pos. 2
                setPrevModelStates();
                setPrevEKFMatrices();
                dyn_wEKFwExtPos2_Estimate(
                    iModelStates_s,
                    iPDyn_wEKFwExtPos2_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s,
                    pTs_d,
                    iPrevPDyn_wEKFwExtPos2_m,
                    iQDyn_wEKFwExtPos2_m,
                    iRDyn_wEKFwExtPos2_m);
                setPrevMeasuredValues();
            } else {
                // Dynamic model with EKF without Ext. Pos.
                setPrevModelStates();
                setPrevEKFMatrices();
                dyn_wEKFwoExtPos_Estimate(
                    iModelStates_s,
                    iPDyn_wEKFwoExtPos_m,
                    iVehicleParameters_s,
                    iMeasuredValues_s,
                    iPrevMeasuredValues_s,
                    iPrevModelStates_s,
                    pTs_d,
                    iPrevPDyn_wEKFwoExtPos_m,
                    iQDyn_wEKFwoExtPos_m,
                    iRDyn_wEKFwoExtPos_m,
                    false);
                setPrevMeasuredValues();
            }
        }
    }
}

void cCombinedVehicleModel::getModelStates(sModelStates* pOutModelStates_s) {
    pOutModelStates_s->iBeta_d                 = iModelStates_s.iBeta_d;
    pOutModelStates_s->iLateralAcceleration_d  = iModelStates_s.iLateralAcceleration_d;
    pOutModelStates_s->iPositionX_d            = iModelStates_s.iPositionX_d;
    pOutModelStates_s->iPositionY_d            = iModelStates_s.iPositionY_d;
    pOutModelStates_s->iYawAngle_d             = iModelStates_s.iYawAngle_d;
    pOutModelStates_s->iYawRate_d              = iModelStates_s.iYawRate_d;
    pOutModelStates_s->iLateralVelocity_d      = iModelStates_s.iLateralVelocity_d;
    pOutModelStates_s->iLongitudinalVelocity_d = iModelStates_s.iLongitudinalVelocity_d;
}

void cCombinedVehicleModel::getPrevModelStates(sModelStates* pOutPrevModelStates_s) {
    pOutPrevModelStates_s->iBeta_d                 = iPrevModelStates_s.iBeta_d;
    pOutPrevModelStates_s->iLateralAcceleration_d  = iPrevModelStates_s.iLateralAcceleration_d;
    pOutPrevModelStates_s->iPositionX_d            = iPrevModelStates_s.iPositionX_d;
    pOutPrevModelStates_s->iPositionY_d            = iPrevModelStates_s.iPositionY_d;
    pOutPrevModelStates_s->iYawAngle_d             = iPrevModelStates_s.iYawAngle_d;
    pOutPrevModelStates_s->iYawRate_d              = iPrevModelStates_s.iYawRate_d;
    pOutPrevModelStates_s->iLateralVelocity_d      = iPrevModelStates_s.iLateralVelocity_d;
    pOutPrevModelStates_s->iLongitudinalVelocity_d = iPrevModelStates_s.iLongitudinalVelocity_d;
}

void cCombinedVehicleModel::setYawAngleStates(double pYawAngle_d) {
    iModelStates_s.iYawAngle_d = pYawAngle_d;
    iPrevModelStates_s.iYawAngle_d = pYawAngle_d;
} 

 double cCombinedVehicleModel::getYawAngle(void) {
    return iModelStates_s.iYawAngle_d;
 }

void cCombinedVehicleModel::setPositionCovariance(double pCovariancePos1X_d, double pCovariancePos1Y_d, double pCovariancePos2X_d, double pCovariancePos2Y_d){
    iRDyn_wEKFwExtPos1_m(3, 3) = pCovariancePos1X_d;
    iRDyn_wEKFwExtPos1_m(4, 4) = pCovariancePos1Y_d;
    iRDyn_wEKFwExtPos2_m(3, 3) = pCovariancePos2X_d;
    iRDyn_wEKFwExtPos2_m(4, 4) = pCovariancePos2Y_d;
    iRKin_wEKFwExtPos1_m(2, 2) = pCovariancePos1X_d;
    iRKin_wEKFwExtPos1_m(3, 3) = pCovariancePos1Y_d;
    iRKin_wEKFwExtPos2_m(2, 2) = pCovariancePos2X_d;
    iRKin_wEKFwExtPos2_m(3, 3) = pCovariancePos2Y_d;
    iRDyn_wEKFwExtPos1Pos2_m(3, 3) = pCovariancePos1X_d;
    iRDyn_wEKFwExtPos1Pos2_m(4, 4) = pCovariancePos1Y_d;
    iRDyn_wEKFwExtPos1Pos2_m(5, 5) = pCovariancePos2X_d;
    iRDyn_wEKFwExtPos1Pos2_m(6, 6) = pCovariancePos2Y_d;
    iRKin_wEKFwExtPos1Pos2_m(2, 2) = pCovariancePos1X_d;
    iRKin_wEKFwExtPos1Pos2_m(3, 3) = pCovariancePos1Y_d;
	iRKin_wEKFwExtPos1Pos2_m(4, 4) = pCovariancePos2X_d;
    iRKin_wEKFwExtPos1Pos2_m(5, 5) = pCovariancePos2Y_d;
}

void cCombinedVehicleModel::setRDyn_wEKFwExtPos1(matrix<double> pR_m) {
    iRDyn_wEKFwExtPos1_m(0, 0) = pR_m(0, 0);
    iRDyn_wEKFwExtPos1_m(0, 1) = pR_m(0, 1);
    iRDyn_wEKFwExtPos1_m(0, 2) = pR_m(0, 2);
    iRDyn_wEKFwExtPos1_m(0, 3) = pR_m(0, 3);
    iRDyn_wEKFwExtPos1_m(0, 4) = pR_m(0, 4);
    iRDyn_wEKFwExtPos1_m(1, 0) = pR_m(1, 0);
    iRDyn_wEKFwExtPos1_m(1, 1) = pR_m(1, 1);
    iRDyn_wEKFwExtPos1_m(1, 2) = pR_m(1, 2);
    iRDyn_wEKFwExtPos1_m(1, 3) = pR_m(1, 3);
    iRDyn_wEKFwExtPos1_m(1, 4) = pR_m(1, 4);
    iRDyn_wEKFwExtPos1_m(2, 0) = pR_m(2, 0);
    iRDyn_wEKFwExtPos1_m(2, 1) = pR_m(2, 1);
    iRDyn_wEKFwExtPos1_m(2, 2) = pR_m(2, 2);
    iRDyn_wEKFwExtPos1_m(2, 3) = pR_m(2, 3);
    iRDyn_wEKFwExtPos1_m(2, 4) = pR_m(2, 4);
    iRDyn_wEKFwExtPos1_m(3, 0) = pR_m(3, 0);
    iRDyn_wEKFwExtPos1_m(3, 1) = pR_m(3, 1);
    iRDyn_wEKFwExtPos1_m(3, 2) = pR_m(3, 2);
    iRDyn_wEKFwExtPos1_m(3, 3) = pR_m(3, 3);
    iRDyn_wEKFwExtPos1_m(3, 4) = pR_m(3, 4);
    iRDyn_wEKFwExtPos1_m(4, 0) = pR_m(4, 0);
    iRDyn_wEKFwExtPos1_m(4, 1) = pR_m(4, 1);
    iRDyn_wEKFwExtPos1_m(4, 2) = pR_m(4, 2);
    iRDyn_wEKFwExtPos1_m(4, 3) = pR_m(4, 3);
    iRDyn_wEKFwExtPos1_m(4, 4) = pR_m(4, 4);
}

void cCombinedVehicleModel::setRKin_wEKFwExtPos1(matrix<double> pR_m) {
    iRKin_wEKFwExtPos1_m(0, 0) = pR_m(0, 0);
    iRKin_wEKFwExtPos1_m(0, 1) = pR_m(0, 1);
    iRKin_wEKFwExtPos1_m(0, 2) = pR_m(0, 2);
    iRKin_wEKFwExtPos1_m(0, 3) = pR_m(0, 3);
    iRKin_wEKFwExtPos1_m(0, 4) = pR_m(0, 4);
    iRKin_wEKFwExtPos1_m(1, 0) = pR_m(1, 0);
    iRKin_wEKFwExtPos1_m(1, 1) = pR_m(1, 1);
    iRKin_wEKFwExtPos1_m(1, 2) = pR_m(1, 2);
    iRKin_wEKFwExtPos1_m(1, 3) = pR_m(1, 3);
    iRKin_wEKFwExtPos1_m(1, 4) = pR_m(1, 4);
    iRKin_wEKFwExtPos1_m(2, 0) = pR_m(2, 0);
    iRKin_wEKFwExtPos1_m(2, 1) = pR_m(2, 1);
    iRKin_wEKFwExtPos1_m(2, 2) = pR_m(2, 2);
    iRKin_wEKFwExtPos1_m(2, 3) = pR_m(2, 3);
    iRKin_wEKFwExtPos1_m(2, 4) = pR_m(2, 4);
    iRKin_wEKFwExtPos1_m(3, 0) = pR_m(3, 0);
    iRKin_wEKFwExtPos1_m(3, 1) = pR_m(3, 1);
    iRKin_wEKFwExtPos1_m(3, 2) = pR_m(3, 2);
    iRKin_wEKFwExtPos1_m(3, 3) = pR_m(3, 3);
    iRKin_wEKFwExtPos1_m(3, 4) = pR_m(3, 4);
    iRKin_wEKFwExtPos1_m(4, 0) = pR_m(4, 0);
    iRKin_wEKFwExtPos1_m(4, 1) = pR_m(4, 1);
    iRKin_wEKFwExtPos1_m(4, 2) = pR_m(4, 2);
    iRKin_wEKFwExtPos1_m(4, 3) = pR_m(4, 3);
    iRKin_wEKFwExtPos1_m(4, 4) = pR_m(4, 4);
}

void cCombinedVehicleModel::setRDyn_wEKFwExtPos2(matrix<double> pR_m) {
    iRDyn_wEKFwExtPos2_m(0, 0) = pR_m(0, 0);
    iRDyn_wEKFwExtPos2_m(0, 1) = pR_m(0, 1);
    iRDyn_wEKFwExtPos2_m(0, 2) = pR_m(0, 2);
    iRDyn_wEKFwExtPos2_m(0, 3) = pR_m(0, 3);
    iRDyn_wEKFwExtPos2_m(0, 4) = pR_m(0, 4);
    iRDyn_wEKFwExtPos2_m(1, 0) = pR_m(1, 0);
    iRDyn_wEKFwExtPos2_m(1, 1) = pR_m(1, 1);
    iRDyn_wEKFwExtPos2_m(1, 2) = pR_m(1, 2);
    iRDyn_wEKFwExtPos2_m(1, 3) = pR_m(1, 3);
    iRDyn_wEKFwExtPos2_m(1, 4) = pR_m(1, 4);
    iRDyn_wEKFwExtPos2_m(2, 0) = pR_m(2, 0);
    iRDyn_wEKFwExtPos2_m(2, 1) = pR_m(2, 1);
    iRDyn_wEKFwExtPos2_m(2, 2) = pR_m(2, 2);
    iRDyn_wEKFwExtPos2_m(2, 3) = pR_m(2, 3);
    iRDyn_wEKFwExtPos2_m(2, 4) = pR_m(2, 4);
    iRDyn_wEKFwExtPos2_m(3, 0) = pR_m(3, 0);
    iRDyn_wEKFwExtPos2_m(3, 1) = pR_m(3, 1);
    iRDyn_wEKFwExtPos2_m(3, 2) = pR_m(3, 2);
    iRDyn_wEKFwExtPos2_m(3, 3) = pR_m(3, 3);
    iRDyn_wEKFwExtPos2_m(3, 4) = pR_m(3, 4);
    iRDyn_wEKFwExtPos2_m(4, 0) = pR_m(4, 0);
    iRDyn_wEKFwExtPos2_m(4, 1) = pR_m(4, 1);
    iRDyn_wEKFwExtPos2_m(4, 2) = pR_m(4, 2);
    iRDyn_wEKFwExtPos2_m(4, 3) = pR_m(4, 3);
    iRDyn_wEKFwExtPos2_m(4, 4) = pR_m(4, 4);
}

void cCombinedVehicleModel::setRKin_wEKFwExtPos2(matrix<double> pR_m) {
    iRKin_wEKFwExtPos2_m(0, 0) = pR_m(0, 0);
    iRKin_wEKFwExtPos2_m(0, 1) = pR_m(0, 1);
    iRKin_wEKFwExtPos2_m(0, 2) = pR_m(0, 2);
    iRKin_wEKFwExtPos2_m(0, 3) = pR_m(0, 3);
    iRKin_wEKFwExtPos2_m(0, 4) = pR_m(0, 4);
    iRKin_wEKFwExtPos2_m(1, 0) = pR_m(1, 0);
    iRKin_wEKFwExtPos2_m(1, 1) = pR_m(1, 1);
    iRKin_wEKFwExtPos2_m(1, 2) = pR_m(1, 2);
    iRKin_wEKFwExtPos2_m(1, 3) = pR_m(1, 3);
    iRKin_wEKFwExtPos2_m(1, 4) = pR_m(1, 4);
    iRKin_wEKFwExtPos2_m(2, 0) = pR_m(2, 0);
    iRKin_wEKFwExtPos2_m(2, 1) = pR_m(2, 1);
    iRKin_wEKFwExtPos2_m(2, 2) = pR_m(2, 2);
    iRKin_wEKFwExtPos2_m(2, 3) = pR_m(2, 3);
    iRKin_wEKFwExtPos2_m(2, 4) = pR_m(2, 4);
    iRKin_wEKFwExtPos2_m(3, 0) = pR_m(3, 0);
    iRKin_wEKFwExtPos2_m(3, 1) = pR_m(3, 1);
    iRKin_wEKFwExtPos2_m(3, 2) = pR_m(3, 2);
    iRKin_wEKFwExtPos2_m(3, 3) = pR_m(3, 3);
    iRKin_wEKFwExtPos2_m(3, 4) = pR_m(3, 4);
    iRKin_wEKFwExtPos2_m(4, 0) = pR_m(4, 0);
    iRKin_wEKFwExtPos2_m(4, 1) = pR_m(4, 1);
    iRKin_wEKFwExtPos2_m(4, 2) = pR_m(4, 2);
    iRKin_wEKFwExtPos2_m(4, 3) = pR_m(4, 3);
    iRKin_wEKFwExtPos2_m(4, 4) = pR_m(4, 4);
}

void cCombinedVehicleModel::setRDyn_wEKFwExtPos1Pos2(matrix<double> pR_m) {
    iRDyn_wEKFwExtPos1Pos2_m(0, 0) = pR_m(0, 0);
    iRDyn_wEKFwExtPos1Pos2_m(0, 1) = pR_m(0, 1);
    iRDyn_wEKFwExtPos1Pos2_m(0, 2) = pR_m(0, 2);
    iRDyn_wEKFwExtPos1Pos2_m(0, 3) = pR_m(0, 3);
    iRDyn_wEKFwExtPos1Pos2_m(0, 4) = pR_m(0, 4);
    iRDyn_wEKFwExtPos1Pos2_m(0, 5) = pR_m(0, 5);
    iRDyn_wEKFwExtPos1Pos2_m(0, 6) = pR_m(0, 6);
    iRDyn_wEKFwExtPos1Pos2_m(1, 0) = pR_m(1, 0);
    iRDyn_wEKFwExtPos1Pos2_m(1, 1) = pR_m(1, 1);
    iRDyn_wEKFwExtPos1Pos2_m(1, 2) = pR_m(1, 2);
    iRDyn_wEKFwExtPos1Pos2_m(1, 3) = pR_m(1, 3);
    iRDyn_wEKFwExtPos1Pos2_m(1, 4) = pR_m(1, 4);
    iRDyn_wEKFwExtPos1Pos2_m(1, 3) = pR_m(1, 5);
    iRDyn_wEKFwExtPos1Pos2_m(1, 4) = pR_m(1, 6);
    iRDyn_wEKFwExtPos1Pos2_m(2, 0) = pR_m(2, 0);
    iRDyn_wEKFwExtPos1Pos2_m(2, 1) = pR_m(2, 1);
    iRDyn_wEKFwExtPos1Pos2_m(2, 2) = pR_m(2, 2);
    iRDyn_wEKFwExtPos1Pos2_m(2, 3) = pR_m(2, 3);
    iRDyn_wEKFwExtPos1Pos2_m(2, 4) = pR_m(2, 4);
    iRDyn_wEKFwExtPos1Pos2_m(2, 3) = pR_m(2, 5);
    iRDyn_wEKFwExtPos1Pos2_m(2, 4) = pR_m(2, 6);
    iRDyn_wEKFwExtPos1Pos2_m(3, 0) = pR_m(3, 0);
    iRDyn_wEKFwExtPos1Pos2_m(3, 1) = pR_m(3, 1);
    iRDyn_wEKFwExtPos1Pos2_m(3, 2) = pR_m(3, 2);
    iRDyn_wEKFwExtPos1Pos2_m(3, 3) = pR_m(3, 3);
    iRDyn_wEKFwExtPos1Pos2_m(3, 4) = pR_m(3, 4);
    iRDyn_wEKFwExtPos1Pos2_m(3, 3) = pR_m(3, 5);
    iRDyn_wEKFwExtPos1Pos2_m(3, 4) = pR_m(3, 6);
    iRDyn_wEKFwExtPos1Pos2_m(4, 0) = pR_m(4, 0);
    iRDyn_wEKFwExtPos1Pos2_m(4, 1) = pR_m(4, 1);
    iRDyn_wEKFwExtPos1Pos2_m(4, 2) = pR_m(4, 2);
    iRDyn_wEKFwExtPos1Pos2_m(4, 3) = pR_m(4, 3);
    iRDyn_wEKFwExtPos1Pos2_m(4, 4) = pR_m(4, 4);
    iRDyn_wEKFwExtPos1Pos2_m(4, 3) = pR_m(4, 5);
    iRDyn_wEKFwExtPos1Pos2_m(4, 4) = pR_m(4, 6);
    iRDyn_wEKFwExtPos1Pos2_m(5, 0) = pR_m(5, 0);
    iRDyn_wEKFwExtPos1Pos2_m(5, 1) = pR_m(5, 1);
    iRDyn_wEKFwExtPos1Pos2_m(5, 2) = pR_m(5, 2);
    iRDyn_wEKFwExtPos1Pos2_m(5, 3) = pR_m(5, 3);
    iRDyn_wEKFwExtPos1Pos2_m(5, 4) = pR_m(5, 4);
    iRDyn_wEKFwExtPos1Pos2_m(5, 5) = pR_m(5, 5);
    iRKin_wEKFwExtPos1Pos2_m(5, 6) = pR_m(5, 6);
    iRDyn_wEKFwExtPos1Pos2_m(6, 0) = pR_m(6, 0);
    iRDyn_wEKFwExtPos1Pos2_m(6, 1) = pR_m(6, 1);
    iRDyn_wEKFwExtPos1Pos2_m(6, 2) = pR_m(6, 2);
    iRDyn_wEKFwExtPos1Pos2_m(6, 3) = pR_m(6, 3);
    iRDyn_wEKFwExtPos1Pos2_m(6, 4) = pR_m(6, 4);
    iRDyn_wEKFwExtPos1Pos2_m(6, 3) = pR_m(6, 5);
    iRDyn_wEKFwExtPos1Pos2_m(6, 4) = pR_m(6, 6);
}

void cCombinedVehicleModel::setRKin_wEKFwExtPos1Pos2(matrix<double> pR_m) {
    iRKin_wEKFwExtPos1Pos2_m(0, 0) = pR_m(0, 0);
    iRKin_wEKFwExtPos1Pos2_m(0, 1) = pR_m(0, 1);
    iRKin_wEKFwExtPos1Pos2_m(0, 2) = pR_m(0, 2);
    iRKin_wEKFwExtPos1Pos2_m(0, 3) = pR_m(0, 3);
    iRKin_wEKFwExtPos1Pos2_m(0, 4) = pR_m(0, 4);
    iRKin_wEKFwExtPos1Pos2_m(0, 5) = pR_m(0, 5);
    iRKin_wEKFwExtPos1Pos2_m(0, 6) = pR_m(0, 6);
    iRKin_wEKFwExtPos1Pos2_m(1, 0) = pR_m(1, 0);
    iRKin_wEKFwExtPos1Pos2_m(1, 1) = pR_m(1, 1);
    iRKin_wEKFwExtPos1Pos2_m(1, 2) = pR_m(1, 2);
    iRKin_wEKFwExtPos1Pos2_m(1, 3) = pR_m(1, 3);
    iRKin_wEKFwExtPos1Pos2_m(1, 4) = pR_m(1, 4);
    iRKin_wEKFwExtPos1Pos2_m(1, 3) = pR_m(1, 5);
    iRKin_wEKFwExtPos1Pos2_m(1, 4) = pR_m(1, 6);
    iRKin_wEKFwExtPos1Pos2_m(2, 0) = pR_m(2, 0);
    iRKin_wEKFwExtPos1Pos2_m(2, 1) = pR_m(2, 1);
    iRKin_wEKFwExtPos1Pos2_m(2, 2) = pR_m(2, 2);
    iRKin_wEKFwExtPos1Pos2_m(2, 3) = pR_m(2, 3);
    iRKin_wEKFwExtPos1Pos2_m(2, 4) = pR_m(2, 4);
    iRKin_wEKFwExtPos1Pos2_m(2, 3) = pR_m(2, 5);
    iRKin_wEKFwExtPos1Pos2_m(2, 4) = pR_m(2, 6);
    iRKin_wEKFwExtPos1Pos2_m(3, 0) = pR_m(3, 0);
    iRKin_wEKFwExtPos1Pos2_m(3, 1) = pR_m(3, 1);
    iRKin_wEKFwExtPos1Pos2_m(3, 2) = pR_m(3, 2);
    iRKin_wEKFwExtPos1Pos2_m(3, 3) = pR_m(3, 3);
    iRKin_wEKFwExtPos1Pos2_m(3, 4) = pR_m(3, 4);
    iRKin_wEKFwExtPos1Pos2_m(3, 3) = pR_m(3, 5);
    iRKin_wEKFwExtPos1Pos2_m(3, 4) = pR_m(3, 6);
    iRKin_wEKFwExtPos1Pos2_m(4, 0) = pR_m(4, 0);
    iRKin_wEKFwExtPos1Pos2_m(4, 1) = pR_m(4, 1);
    iRKin_wEKFwExtPos1Pos2_m(4, 2) = pR_m(4, 2);
    iRKin_wEKFwExtPos1Pos2_m(4, 3) = pR_m(4, 3);
    iRKin_wEKFwExtPos1Pos2_m(4, 4) = pR_m(4, 4);
    iRKin_wEKFwExtPos1Pos2_m(4, 3) = pR_m(4, 5);
    iRKin_wEKFwExtPos1Pos2_m(4, 4) = pR_m(4, 6);
    iRKin_wEKFwExtPos1Pos2_m(5, 0) = pR_m(5, 0);
    iRKin_wEKFwExtPos1Pos2_m(5, 1) = pR_m(5, 1);
    iRKin_wEKFwExtPos1Pos2_m(5, 2) = pR_m(5, 2);
    iRKin_wEKFwExtPos1Pos2_m(5, 3) = pR_m(5, 3);
    iRKin_wEKFwExtPos1Pos2_m(5, 4) = pR_m(5, 4);
    iRKin_wEKFwExtPos1Pos2_m(5, 5) = pR_m(5, 5);
    iRKin_wEKFwExtPos1Pos2_m(5, 6) = pR_m(5, 6);
    iRKin_wEKFwExtPos1Pos2_m(6, 0) = pR_m(6, 0);
    iRKin_wEKFwExtPos1Pos2_m(6, 1) = pR_m(6, 1);
    iRKin_wEKFwExtPos1Pos2_m(6, 2) = pR_m(6, 2);
    iRKin_wEKFwExtPos1Pos2_m(6, 3) = pR_m(6, 3);
    iRKin_wEKFwExtPos1Pos2_m(6, 4) = pR_m(6, 4);
    iRKin_wEKFwExtPos1Pos2_m(6, 3) = pR_m(6, 5);
    iRKin_wEKFwExtPos1Pos2_m(6, 4) = pR_m(6, 6);
}

void cCombinedVehicleModel::setRDyn_wEKFwoExtPos(matrix<double> pR_m) {
    iRDyn_wEKFwoExtPos_m(0, 0) = pR_m(0, 0);
    iRDyn_wEKFwoExtPos_m(0, 1) = pR_m(0, 1);
    iRDyn_wEKFwoExtPos_m(0, 2) = pR_m(0, 2);
    iRDyn_wEKFwoExtPos_m(1, 0) = pR_m(1, 0);
    iRDyn_wEKFwoExtPos_m(1, 1) = pR_m(1, 1);
    iRDyn_wEKFwoExtPos_m(1, 2) = pR_m(1, 2);
    iRDyn_wEKFwoExtPos_m(2, 0) = pR_m(2, 0);
    iRDyn_wEKFwoExtPos_m(2, 1) = pR_m(2, 1);
    iRDyn_wEKFwoExtPos_m(2, 2) = pR_m(2, 2);
}

void cCombinedVehicleModel::setRKin_wEKFwoExtPos(matrix<double> pR_m) {
    iRKin_wEKFwoExtPos_m(0, 0) = pR_m(0, 0);
    iRKin_wEKFwoExtPos_m(0, 1) = pR_m(0, 1);
    iRKin_wEKFwoExtPos_m(0, 2) = pR_m(0, 2);
    iRKin_wEKFwoExtPos_m(1, 0) = pR_m(1, 0);
    iRKin_wEKFwoExtPos_m(1, 1) = pR_m(1, 1);
    iRKin_wEKFwoExtPos_m(1, 2) = pR_m(1, 2);
    iRKin_wEKFwoExtPos_m(2, 0) = pR_m(2, 0);
    iRKin_wEKFwoExtPos_m(2, 1) = pR_m(2, 1);
    iRKin_wEKFwoExtPos_m(2, 2) = pR_m(2, 2);
}

