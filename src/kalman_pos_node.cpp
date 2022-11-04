#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <autoware_msgs/VehicleStatus.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>
#include <novatel_gps_msgs/Inspvax.h>
#include "visualization_msgs/Marker.h"

//#include "geometry_msgs/Vector3.h"
//#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"

#include <tf2/buffer_core.h>

#include "CombinedVehicleModel.h"

geometry_msgs::PoseStamped orig_pose_;

sensor_msgs::NavSatFix gNavSatFixMsg;
geometry_msgs::PoseStamped gPositionMsg;
geometry_msgs::PoseStamped gCoGPositionMsg;
sensor_msgs::Imu gIMUMsg;
autoware_msgs::VehicleStatus gVehicleStatusMsg;
novatel_gps_msgs::Inspvax gNovatelStatus;
std_msgs::String gDuroStatusMsg;

bool gPoseMsgArrived_b = false;
bool gNavSatFixMsgArrived_b = false;
bool gIMUMsgArrived_b = false;
bool gVehicleStatusMsgArrived_b = false;
bool gNovatelStatusMsgArrived_b = false;
bool gDuroStatusMsgArrived_b = false;

std::string gVehicleType = "leaf";
std::string gGnssSource = "nova";

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gPoseMsgArrived_b = true;
    orig_pose_ = *msg;
    gPositionMsg = *msg;

    if (gGnssSource == "nova") {
        if (gVehicleType == "leaf") {
            tf2::BufferCore lBufferCore;

            geometry_msgs::TransformStamped lNovaTransform;
            lNovaTransform.header.frame_id = "map";
            lNovaTransform.child_frame_id = "nova";
            lNovaTransform.transform.translation.x = gPositionMsg.pose.position.x;
            lNovaTransform.transform.translation.y = gPositionMsg.pose.position.y;
            lNovaTransform.transform.translation.z = gPositionMsg.pose.position.z;
            lNovaTransform.transform.rotation = gPositionMsg.pose.orientation;
            lBufferCore.setTransform(lNovaTransform, "default_authority", true);

            geometry_msgs::TransformStamped lCoGTransform;
            lCoGTransform.header.frame_id = "nova";
            lCoGTransform.child_frame_id = "cog";
            lCoGTransform.transform.translation.x = 0.208;
            lCoGTransform.transform.translation.y = -0.408;
            lCoGTransform.transform.translation.z = -1.278;
            lCoGTransform.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            lBufferCore.setTransform(lCoGTransform, "default_authority", true);

            geometry_msgs::TransformStamped lTsLookup;
            lTsLookup = lBufferCore.lookupTransform("cog", "map", ros::Time(0));
            gCoGPositionMsg.pose.position.x = lTsLookup.transform.translation.x;
            gCoGPositionMsg.pose.position.y = lTsLookup.transform.translation.y;
            gCoGPositionMsg.pose.position.z = lTsLookup.transform.translation.z;
            gCoGPositionMsg.pose.orientation= lTsLookup.transform.rotation;
        } else {
            gCoGPositionMsg = gPositionMsg;
        }
    } else {
        gCoGPositionMsg = gPositionMsg;
    }
    //ROS_INFO_STREAM("x: " << msg->pose.position.x << " y: " << msg->pose.position.y);
}

void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gNavSatFixMsgArrived_b = true;
    gNavSatFixMsg = *msg;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    gIMUMsgArrived_b = true;
    gIMUMsg = *msg;
    //ROS_INFO_STREAM("acc x: " << msg->linear_acceleration.x << " y: " << msg->linear_acceleration.y);
}

void vehicleCallback(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
    gVehicleStatusMsgArrived_b = true;
    gVehicleStatusMsg = *msg;
  //ROS_INFO_STREAM("speed: " << msg->speed << " angle: " << msg->angle);
}

void novatelStatusCallback(const novatel_gps_msgs::Inspvax::ConstPtr& msg)
{
    gNovatelStatusMsgArrived_b = true;
    gNovatelStatus = *msg;
}

void duroStatusStringCallback(const std_msgs::String::ConstPtr& msg)
{
    gDuroStatusMsgArrived_b = true;
    gDuroStatusMsg = *msg;
}

int main(int argc, char **argv)
{
    std::string pose_topic, imu_topic, estimated_pose, estimated_debug_pose, estimatation_accuracy, nav_sat_fix_topic, duro_status_string_topic, inspvax_topic, vehicle_status_topic;
    bool debug;
    int loop_rate_hz;
    int estimation_method;
    cCombinedVehicleModel lCombinedVehicleModel("leaf");
    bool lFirstIteration = true;

    ros::init(argc, argv, "kalman_pos_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("pose_topic", pose_topic, "gps/nova/current_pose");
    n_private.param<std::string>("vehicle_status_topic", vehicle_status_topic, "vehicle_status");
    n_private.param<std::string>("nav_sat_fix_topic", nav_sat_fix_topic, "gps/nova/fix");
    n_private.param<std::string>("duro_status_string_topic", duro_status_string_topic, "gps/duro/status_sring");
    n_private.param<std::string>("inspvax_topic", inspvax_topic, "gps/nova/inspvax" );
    n_private.param<std::string>("imu_topic", imu_topic, "gps/nova/imu");
    n_private.param<std::string>("est_topic", estimated_pose, "estimated_pose");
    n_private.param<std::string>("est_debug_topic", estimated_debug_pose, "estimated_debug_pose");
    n_private.param<std::string>("est_accuracy_topic", estimatation_accuracy, "estimation_accuracy");
    n_private.param<std::string>("vehicle_type", gVehicleType, "leaf");
    n_private.param<bool>("debug", debug, false);
    n_private.param<int>("loop_rate_hz", loop_rate_hz, 10);
    n_private.param<int>("estimation_method", estimation_method, 0);
    n_private.param<std::string>("gnss_source", gGnssSource, "nova");
    ROS_INFO_STREAM("kalman_pos_node started | pose: " << pose_topic 
                    << " | debug: " << debug 
                    << " | nav_sat_fix: " << nav_sat_fix_topic
                    << " | imu: " << imu_topic
                    << " | est: " << estimated_pose
                    << " | est_debug: " << estimated_debug_pose
                    << " | est_accuracy: " << estimatation_accuracy
                    << " | vehicle_type: " << gVehicleType
                    << " | loop_rate_hz: " << loop_rate_hz
                    << " | estimation_method: " << estimation_method
                    << " | gnss_source: " << gGnssSource );
    

    ros::Publisher accuracy_marker_pub = n.advertise<visualization_msgs::Marker>( estimatation_accuracy, 1000 );
    ros::Publisher est_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_pose, 1000);
    ros::Publisher est_debug_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_debug_pose, 1000);
    ros::Subscriber sub_pose = n.subscribe(pose_topic, 1000, poseCallback);
    if (gGnssSource == "nova"){
        ros::Subscriber sub_nav_sat_fix = n.subscribe(nav_sat_fix_topic, 1000, navSatFixCallback);
    }
    ros::Subscriber sub_imu = n.subscribe(imu_topic, 1000, imuCallback);
    ros::Subscriber sub_vehicle = n.subscribe(vehicle_status_topic, 1000, vehicleCallback);
    if (gGnssSource == "nova") {
        ros::Subscriber sub_inspvax = n.subscribe(inspvax_topic, 1000, novatelStatusCallback);
    }
    if ((gGnssSource == "duro") || (gVehicleType == "SZEmission")) {
        ros::Subscriber sub_durostatus = n.subscribe(duro_status_string_topic, 1000, duroStatusStringCallback);
    }
    ros::Rate loop_rate(loop_rate_hz); // 10 Hz

    gPoseMsgArrived_b = false;
    gNavSatFixMsgArrived_b = false;
    gIMUMsgArrived_b = false;
    gVehicleStatusMsgArrived_b = false;
    gNovatelStatusMsgArrived_b = false;
    gDuroStatusMsgArrived_b = false;

    lCombinedVehicleModel.initVehicleParameters(gVehicleType);
    lCombinedVehicleModel.initEKFMatrices();

    ROS_INFO_STREAM("ROS::OK  " << ros::ok());

    while (ros::ok())
    {
        visualization_msgs::Marker est_accuracy_marker;
        geometry_msgs::PoseStamped est_pose_msg;
        geometry_msgs::PoseStamped est_debug_pose_msg;

        eEstimationMode lEstimationMode_e = eEstimationMode::ekf;
        eGNSSState lGNSSState_e = eGNSSState::off;

        int lAccuracyScaleFactor = 10;
    
        //ROS_INFO_STREAM("Pose Arrived " << gPoseMsgArrived_b << "  | IMU Arrived: " << gIMUMsgArrived_b << " | Status Arrived: " << gVehicleStatusMsgArrived_b);
        if (gPoseMsgArrived_b && gIMUMsgArrived_b && gVehicleStatusMsgArrived_b) {
            
            sModelStates lCurrentModelStates_s;

            double lTmpRoll_d;
            double lTmpPitch_d;
            double lTmpYaw_d;

            tf::Quaternion lTmpQuternion_c(
                gCoGPositionMsg.pose.orientation.x,
                gCoGPositionMsg.pose.orientation.y,
                gCoGPositionMsg.pose.orientation.z,
                gCoGPositionMsg.pose.orientation.w);
            tf::Matrix3x3 lTmpMatrix(lTmpQuternion_c);

            lTmpMatrix.getRPY(lTmpRoll_d, lTmpPitch_d, lTmpYaw_d);

            //lCombinedVehicleModel.setPrevMeasuredValues();
            lCombinedVehicleModel.setMeasuredValuesGNSS(gCoGPositionMsg.pose.position.x, gCoGPositionMsg.pose.position.y, gCoGPositionMsg.pose.position.z, lTmpYaw_d);
            lCombinedVehicleModel.setMeasuredValuesIMU(gIMUMsg.linear_acceleration.x, gIMUMsg.linear_acceleration.y, gIMUMsg.linear_acceleration.z, gIMUMsg.angular_velocity.x, gIMUMsg.angular_velocity.y, gIMUMsg.angular_velocity.z);
            lCombinedVehicleModel.setMeasuredValuesVehicleState(gVehicleStatusMsg.angle, gVehicleStatusMsg.speed);
                
            if (lFirstIteration) {
                lCombinedVehicleModel.setPrevEKFMatrices();
                lCombinedVehicleModel.setPrevMeasuredValues();
                lCombinedVehicleModel.setModelStates(0, gIMUMsg.angular_velocity.z, lTmpYaw_d, gIMUMsg.linear_acceleration.y, gCoGPositionMsg.pose.position.x, gCoGPositionMsg.pose.position.y, gVehicleStatusMsg.speed, 0);
                lFirstIteration = false;
            }

            if (gGnssSource == "nova") {
                if ((!gNovatelStatusMsgArrived_b) || (estimation_method == 0)) {
                    lEstimationMode_e = eEstimationMode::model;
                    lGNSSState_e = eGNSSState::off;
                    lAccuracyScaleFactor = 10;
                    //ROS_INFO_STREAM(1);
                } else {
                    if(gNovatelStatus.position_type ==  "NONE"){ //POSITION_TYPE_NONE
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                        //ROS_INFO_STREAM(1);
                    }
                    else if(gNovatelStatus.position_type ==  "INS_SBAS"){ //POSITION_TYPE_SBAS
                        lEstimationMode_e = eEstimationMode::ekf;
                        lGNSSState_e = eGNSSState::rtk_float;
                        //lEstimationMode_e = eEstimationMode::model;
                        //lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                        //ROS_INFO_STREAM("INS_SBAS");
                    }
                    else if(gNovatelStatus.position_type ==  "SINGLE"){ //POSITION_TYPE_PSEUDORANGE_SINGLE_POINT
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                        //ROS_INFO_STREAM(3);
                    }
                    else if(gNovatelStatus.position_type ==  "PSRDIFF"){ //POSITION_TYPE_PSEUDORANGE_DIFFERENTIAL
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                        //ROS_INFO_STREAM(4);
                    }
                    else if(gNovatelStatus.position_type ==  "INS_RTKFLOAT"){ //POSITION_TYPE_RTK_FLOAT
                        lEstimationMode_e = eEstimationMode::ekf;
                        lGNSSState_e = eGNSSState::rtk_float;
                        lAccuracyScaleFactor = 2.5;
                        //ROS_INFO_STREAM(5);
                    }
                    else if(gNovatelStatus.position_type ==  "INS_RTKFIXED"){ //POSITION_TYPE_RTK_FIXED
                        lEstimationMode_e = eEstimationMode::ekf;
                        lGNSSState_e = eGNSSState::rtk_fixed;
                        lAccuracyScaleFactor = 1;
                        //ROS_INFO_STREAM(6);
                    }
                    else{
                        lEstimationMode_e = eEstimationMode::model;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                        //ROS_INFO_STREAM("else " << gNovatelStatus.position_type);
                    }
                }
            } else if (gGnssSource == "duro") {
                if ((!gDuroStatusMsgArrived_b) || (estimation_method == 0)){
                    lEstimationMode_e = eEstimationMode::model;
                    lGNSSState_e = eGNSSState::off;
                    lAccuracyScaleFactor = 10;
                } else {
                    if (gDuroStatusMsg.data == "Invalid") {
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                    } else if (gDuroStatusMsg.data == "Single Point Position (SPP)") {
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                    }  else if (gDuroStatusMsg.data == "Differential GNSS (DGNSS)") {
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                    }  else if (gDuroStatusMsg.data == "Float RTK") {
                        lEstimationMode_e = eEstimationMode::ekf;
                        lGNSSState_e = eGNSSState::rtk_float;
                        lAccuracyScaleFactor = 2.5;
                    }  else if (gDuroStatusMsg.data == "Fixed RTK") {
                        lEstimationMode_e = eEstimationMode::ekf;
                        lGNSSState_e = eGNSSState::rtk_fixed;
                        lAccuracyScaleFactor = 1;
                    }  else if (gDuroStatusMsg.data == "Dead Reckoning (DR)") {
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                    }  else if (gDuroStatusMsg.data == "SBAS Position") {
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::pseudorange;
                        lAccuracyScaleFactor = 5;
                    } else {
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                    }
                }
            } else {
                switch (estimation_method) {
                    case 0: 
                        lEstimationMode_e = eEstimationMode::model;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                        break;
                    case 1: 
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                        break;
                    case 10: 
                        if (gVehicleType == "SZEmission") {
                            if ((!gDuroStatusMsgArrived_b)){
                                lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;;
                                lGNSSState_e = eGNSSState::off;
                                lAccuracyScaleFactor = 10;
                            } else {
                                if (gDuroStatusMsg.data == "Invalid") {
                                    lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                                    lGNSSState_e = eGNSSState::off;
                                    lAccuracyScaleFactor = 10;
                                } else if (gDuroStatusMsg.data == "Single Point Position (SPP)") {
                                    lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                                    lGNSSState_e = eGNSSState::pseudorange;
                                    lAccuracyScaleFactor = 5;
                                }  else if (gDuroStatusMsg.data == "Differential GNSS (DGNSS)") {
                                    lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                                    lGNSSState_e = eGNSSState::pseudorange;
                                    lAccuracyScaleFactor = 5;
                                }  else if (gDuroStatusMsg.data == "Float RTK") {
                                    lEstimationMode_e = eEstimationMode::ekf;
                                    lGNSSState_e = eGNSSState::rtk_float;
                                    lAccuracyScaleFactor = 2.5;
                                }  else if (gDuroStatusMsg.data == "Fixed RTK") {
                                    lEstimationMode_e = eEstimationMode::ekf;
                                    lGNSSState_e = eGNSSState::rtk_fixed;
                                    lAccuracyScaleFactor = 1;
                                }  else if (gDuroStatusMsg.data == "Dead Reckoning (DR)") {
                                    lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                                    lGNSSState_e = eGNSSState::pseudorange;
                                    lAccuracyScaleFactor = 5;
                                }  else if (gDuroStatusMsg.data == "SBAS Position") {
                                    lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                                    lGNSSState_e = eGNSSState::pseudorange;
                                    lAccuracyScaleFactor = 5;
                                } else {
                                    lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                                    lGNSSState_e = eGNSSState::off;
                                    lAccuracyScaleFactor = 10;
                                }
                            }
                        } else {
                            lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                            lGNSSState_e = eGNSSState::off;
                            lAccuracyScaleFactor = 10;
                        }
                        break;
                    default:
                        //lEstimationMode_e = eEstimationMode::model;
                        lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
                        lGNSSState_e = eGNSSState::off;
                        lAccuracyScaleFactor = 10;
                        break;
                }
            }

            lCombinedVehicleModel.iterateModel(1.0/loop_rate_hz, lEstimationMode_e, lGNSSState_e);
            lCombinedVehicleModel.getModelStates(&lCurrentModelStates_s);

            est_pose_msg.header.stamp = ros::Time::now();
            est_pose_msg.header.frame_id = "map";
            est_pose_msg.pose.position.x = lCurrentModelStates_s.positionX_d;
            est_pose_msg.pose.position.y = lCurrentModelStates_s.positionY_d;
            est_pose_msg.pose.position.z = 0;
        }
        else {
            est_pose_msg.header.stamp = ros::Time::now();
            est_pose_msg.header.frame_id = "map";
            est_pose_msg.pose.position.x = 0;
            est_pose_msg.pose.position.y = 0;
            est_pose_msg.pose.position.z = 0;
        }

        est_pub.publish(est_pose_msg);

        est_accuracy_marker.header.frame_id = "base_link";
        est_accuracy_marker.header.stamp = ros::Time();
        est_accuracy_marker.ns = "est_pose";
        est_accuracy_marker.id = 0;
        est_accuracy_marker.type = visualization_msgs::Marker::SPHERE;
        est_accuracy_marker.action = visualization_msgs::Marker::ADD;
        est_accuracy_marker.pose.position.x = 0.0;
        est_accuracy_marker.pose.position.y = 0.0;
        est_accuracy_marker.pose.position.z = 0.6;
        /*
        est_accuracy_marker.pose.position.x = est_pose_msg.pose.position.x;
        est_accuracy_marker.pose.position.y = est_pose_msg.pose.position.y;
        est_accuracy_marker.pose.position.z = est_pose_msg.pose.position.z;
        */
        est_accuracy_marker.pose.orientation.x = 0.0;
        est_accuracy_marker.pose.orientation.y = 0.0;
        est_accuracy_marker.pose.orientation.z = 0.0;
        est_accuracy_marker.pose.orientation.w = 1.0;
        if (gNavSatFixMsgArrived_b) {
            est_accuracy_marker.scale.x = lAccuracyScaleFactor * (sqrt(gNavSatFixMsg.position_covariance[0] * gNavSatFixMsg.position_covariance[4]))/2;
            est_accuracy_marker.scale.y = lAccuracyScaleFactor * (sqrt(gNavSatFixMsg.position_covariance[0] * gNavSatFixMsg.position_covariance[4]))/2;
            est_accuracy_marker.scale.z = lAccuracyScaleFactor * (sqrt(gNavSatFixMsg.position_covariance[0] * gNavSatFixMsg.position_covariance[4]))/2;            
        } else {
            est_accuracy_marker.scale.x = lAccuracyScaleFactor * 10;
            est_accuracy_marker.scale.y = lAccuracyScaleFactor * 10;
            est_accuracy_marker.scale.z = lAccuracyScaleFactor * 10;
        }
        est_accuracy_marker.color.a = 0.3; // Don't forget to set the alpha!
        est_accuracy_marker.color.r = 0.1;
        est_accuracy_marker.color.g = 0.8;
        est_accuracy_marker.color.b = 0.4;
        //only if using a MESH_RESOURCE marker type:
        //est_accuracy_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        accuracy_marker_pub.publish( est_accuracy_marker );


      if (debug){
        est_debug_pose_msg.header.stamp = ros::Time::now();
        est_debug_pub.publish(est_debug_pose_msg);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }


    return 0;
}