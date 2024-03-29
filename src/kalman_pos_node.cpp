#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <autoware_msgs/VehicleStatus.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <sstream>
#include <novatel_gps_msgs/Inspvax.h>
#include "visualization_msgs/Marker.h"

#include "tf/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "PositionEstimation.h"

sensor_msgs::NavSatFix gROSNavSatFixMsg_msg;
geometry_msgs::PoseStamped gROSPositionMsg_msg;
geometry_msgs::PoseStamped gROSCogPositionMsg_msg;
sensor_msgs::Imu gROSIMUMsg_msg;
autoware_msgs::VehicleStatus gROSVehicleStatusMsg_msg;
novatel_gps_msgs::Inspvax gROSNovatelStatusMsg_msg;
std_msgs::String gROSDuroStatusMsg_msg;

bool gPoseMsgArrived_b = false;
bool gNavSatFixMsgArrived_b = false;
bool gIMUMsgArrived_b = false;
bool gVehicleStatusMsgArrived_b = false;
bool gNovatelStatusMsgArrived_b = false;
bool gDuroStatusMsgArrived_b = false;
unsigned long long gIMUMsgArriveTime_u64 = 0;
unsigned long long gVehicleStatusMsgArriveTime_u64 = 0;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gPoseMsgArrived_b = true;
    gROSPositionMsg_msg = *msg;
    gROSCogPositionMsg_msg = gROSPositionMsg_msg;
}

void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& pMsg_msgp)
{
    gNavSatFixMsgArrived_b = true;
    gROSNavSatFixMsg_msg = *pMsg_msgp;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& pMsg_msgp)
{
    struct timeval lTimeval_tv;
    gIMUMsgArrived_b = true;
    gROSIMUMsg_msg = *pMsg_msgp;
    gettimeofday(&lTimeval_tv, NULL);
    gIMUMsgArriveTime_u64 = 
        (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
        (unsigned long long)(lTimeval_tv.tv_usec) / 1000;
}

void vehicleCallback(const autoware_msgs::VehicleStatus::ConstPtr& pMsg_msgp)
{
    struct timeval lTimeval_tv;
    gVehicleStatusMsgArrived_b = true;
    gROSVehicleStatusMsg_msg = *pMsg_msgp;
    gettimeofday(&lTimeval_tv, NULL);
    gVehicleStatusMsgArriveTime_u64 = 
        (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
        (unsigned long long)(lTimeval_tv.tv_usec) / 1000;
}

void novatelStatusCallback(const novatel_gps_msgs::Inspvax::ConstPtr& pMsg_msgp)
{
    gNovatelStatusMsgArrived_b = true;
    gROSNovatelStatusMsg_msg = *pMsg_msgp;
}

void duroStatusStringCallback(const std_msgs::String::ConstPtr& pMsg_msgp)
{
    gDuroStatusMsgArrived_b = true;
    gROSDuroStatusMsg_msg = *pMsg_msgp;
}

int main(int argc, char **argv)
{
    // Declare ROS parameter variables
    std::string lROSParamPoseTopic_s;
    std::string lROSParamImuTopic_s;
    std::string lROSParamEstimatedPoseCogTopic_s; 
    std::string lROSParamEstimatedPoseBaselinkTopic_s;
    std::string lROSParamEstimatedTravDistOdom_s;
    std::string lROSParamEstimatedTravDistEstPos_s;
    std::string lROSParamEstimationAccuracyTopic_s;
    std::string lROSParamNavSatFixTopic_s;
    std::string lROSParamDuroStatusStringTopic_s; 
    std::string lROSParamInspvaxTopic_s; 
    std::string lROSParamVehicleStatusTopic_s;
    std::string lROSParamVehicleType_s;
    std::string lROSParamGnssSource_s;
    bool lROSParamDynamicTimeCalcEnabled_b; 
    bool lROSParamDoNotWaitForGnssMsgs_b;
    int lROSParamLoopRateHz_i32;
    int lROSParamEstimationMethod_i32;
    double lROSParamKinematicModelMaxSpeed_d;
    double lROSParamMsgTimeout_d;
    double lROSParamVehicleParamC1_d;
    double lROSParamVehicleParamC2_d;
    double lROSParamVehicleParamM_d;
    double lROSParamVehicleParamJz_d;
    double lROSParamVehicleParamL1_d;
    double lROSParamVehicleParamL2_d;
    double lROSParamVehicleParamSWR_d;

    // Initialize ROS node
    ros::init(argc, argv, "kalman_pos_node");
    ros::NodeHandle lROSNodeHandle_cl;
    ros::NodeHandle lROSNodeHandlePrivate_cl("~");
    lROSNodeHandlePrivate_cl.param<std::string>("pose_topic", lROSParamPoseTopic_s, "gps/nova/current_pose");
    lROSNodeHandlePrivate_cl.param<std::string>("vehicle_status_topic", lROSParamVehicleStatusTopic_s, "vehicle_status");
    lROSNodeHandlePrivate_cl.param<std::string>("nav_sat_fix_topic", lROSParamNavSatFixTopic_s, "gps/nova/fix");
    lROSNodeHandlePrivate_cl.param<std::string>("duro_status_string_topic", lROSParamDuroStatusStringTopic_s, "gps/duro/status_string");
    lROSNodeHandlePrivate_cl.param<std::string>("inspvax_topic", lROSParamInspvaxTopic_s, "gps/nova/inspvax" );
    lROSNodeHandlePrivate_cl.param<std::string>("imu_topic", lROSParamImuTopic_s, "gps/nova/imu");
    lROSNodeHandlePrivate_cl.param<std::string>("est_cog_topic", lROSParamEstimatedPoseCogTopic_s, "estimated_pose_cog");
    lROSNodeHandlePrivate_cl.param<std::string>("est_baselink_topic", lROSParamEstimatedPoseBaselinkTopic_s, "estimated_pose_baselink");
    lROSNodeHandlePrivate_cl.param<std::string>("est_accuracy_topic", lROSParamEstimationAccuracyTopic_s, "estimation_accuracy");
    lROSNodeHandlePrivate_cl.param<std::string>("est_trav_distance_odom_topic", lROSParamEstimatedTravDistOdom_s, "distance");
    lROSNodeHandlePrivate_cl.param<std::string>("est_trav_distance_est_pos_topic", lROSParamEstimatedTravDistEstPos_s, "estimated_trav_dist_est_pos");
    lROSNodeHandlePrivate_cl.param<std::string>("vehicle_type", lROSParamVehicleType_s, "leaf");
    lROSNodeHandlePrivate_cl.param<int>("loop_rate_hz", lROSParamLoopRateHz_i32, 10);
    lROSNodeHandlePrivate_cl.param<int>("estimation_method", lROSParamEstimationMethod_i32, 0);
    lROSNodeHandlePrivate_cl.param<std::string>("gnss_source", lROSParamGnssSource_s, "nova");
    lROSNodeHandlePrivate_cl.param<bool>("dynamic_time_calc", lROSParamDynamicTimeCalcEnabled_b, false);
    lROSNodeHandlePrivate_cl.param<bool>("do_not_wait_for_gnss_msgs", lROSParamDoNotWaitForGnssMsgs_b, false);
    lROSNodeHandlePrivate_cl.param<double>("kinematic_model_max_speed", lROSParamKinematicModelMaxSpeed_d, 0.5);
    lROSNodeHandlePrivate_cl.param<double>("msg_timeout", lROSParamMsgTimeout_d, 2000);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_c1", lROSParamVehicleParamC1_d, 3000);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_c2", lROSParamVehicleParamC2_d, 3000);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_m", lROSParamVehicleParamM_d, 180);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_jz", lROSParamVehicleParamJz_d, 270);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_l1", lROSParamVehicleParamL1_d, 0.324);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_l2", lROSParamVehicleParamL2_d, 0.976);
    lROSNodeHandlePrivate_cl.param<double>("vehicle_param_swr", lROSParamVehicleParamSWR_d, 1);
    

    ROS_INFO_STREAM("kalman_pos_node started | pose: " << lROSParamPoseTopic_s
                    << " | vehicle_status: " << lROSParamVehicleStatusTopic_s 
                    << " | nav_sat_fix: " << lROSParamNavSatFixTopic_s
                    << " | duro_status_string: " << lROSParamDuroStatusStringTopic_s
                    << " | imu: " << lROSParamImuTopic_s
                    << " | est_cog: " << lROSParamEstimatedPoseCogTopic_s
                    << " | est_bl: " << lROSParamEstimatedPoseBaselinkTopic_s
                    << " | est_accuracy: " << lROSParamEstimationAccuracyTopic_s
                    << " | vehicle_type: " << lROSParamVehicleType_s
                    << " | loop_rate_hz: " << lROSParamLoopRateHz_i32
                    << " | estimation_method: " << lROSParamEstimationMethod_i32
                    << " | gnss_source: " << lROSParamGnssSource_s 
                    << " | dynamic_time_calc_enabled: " << lROSParamDynamicTimeCalcEnabled_b
                    << " | vehicle_param_c1: " <<  lROSParamVehicleParamC1_d
                    << " | vehicle_param_c2: " <<  lROSParamVehicleParamC2_d
                    << " | vehicle_param_m: " <<  lROSParamVehicleParamM_d
                    << " | vehicle_param_jz: " <<  lROSParamVehicleParamJz_d
                    << " | vehicle_param_l1: " <<  lROSParamVehicleParamL1_d
                    << " | vehicle_param_l2: " <<  lROSParamVehicleParamL2_d
                    << " | vehicle_param_swr: " <<  lROSParamVehicleParamSWR_d);
    
    ros::Publisher lROSPubAccuracyMarker_cl = lROSNodeHandle_cl.advertise<visualization_msgs::Marker>( lROSParamEstimationAccuracyTopic_s, 1000 );
    ros::Publisher lROSPubEstimatedPoseCog_cl = lROSNodeHandle_cl.advertise<geometry_msgs::PoseStamped>(lROSParamEstimatedPoseCogTopic_s, 1000);
    ros::Publisher lROSPubEstimatedPoseBaselink_cl = lROSNodeHandle_cl.advertise<geometry_msgs::PoseStamped>(lROSParamEstimatedPoseBaselinkTopic_s, 1000);
    ros::Publisher lROSPubEstimatedTraveledDistanceOdom_cl = lROSNodeHandle_cl.advertise<std_msgs::Float32>(lROSParamEstimatedTravDistOdom_s, 1000);
    ros::Publisher lROSPubEstimatedTraveledDistanceEstPos_cl = lROSNodeHandle_cl.advertise<std_msgs::Float32>(lROSParamEstimatedTravDistEstPos_s, 1000);
    
    ros::Subscriber lROSSubPose_cl = lROSNodeHandle_cl.subscribe(lROSParamPoseTopic_s, 1000, poseCallback);
    if (lROSParamGnssSource_s == "nova"){
        ros::Subscriber lROSSubNavSatFix_cl = lROSNodeHandle_cl.subscribe(lROSParamNavSatFixTopic_s, 1000, navSatFixCallback);
    }
    ros::Subscriber lROSSubIMU_cl = lROSNodeHandle_cl.subscribe(lROSParamImuTopic_s, 1000, imuCallback);
    ros::Subscriber lROSSubVehicleStatus_cl = lROSNodeHandle_cl.subscribe(lROSParamVehicleStatusTopic_s, 1000, vehicleCallback);
    if (lROSParamGnssSource_s == "nova") {
        ros::Subscriber lROSSubInspvax_cl = lROSNodeHandle_cl.subscribe(lROSParamInspvaxTopic_s, 1000, novatelStatusCallback);
    }
    //if ((lROSParamGnssSource_s == "duro") || (lROSParamVehicleType_s == "SZEmission")) {
        ros::Subscriber lROSSubDuroStatus_cl = lROSNodeHandle_cl.subscribe(lROSParamDuroStatusStringTopic_s, 1000, duroStatusStringCallback);
    //}
    
    ros::Rate lROSLoopRate_cl(lROSParamLoopRateHz_i32); // 10 Hz
    
    tf2_ros::TransformBroadcaster lEstPosBaselinkTransformBroadcaster_cl;

    gPoseMsgArrived_b = false;
    gNavSatFixMsgArrived_b = false;
    gIMUMsgArrived_b = false;
    gVehicleStatusMsgArrived_b = false;
    gNovatelStatusMsgArrived_b = false;
    gDuroStatusMsgArrived_b = false;

    sVehicleParameters lVehicleParameters_s;
    lVehicleParameters_s.c1_d   = lROSParamVehicleParamC1_d;
    lVehicleParameters_s.c2_d   = lROSParamVehicleParamC2_d;
    lVehicleParameters_s.m_d    = lROSParamVehicleParamM_d;
    lVehicleParameters_s.jz_d   = lROSParamVehicleParamJz_d;
    lVehicleParameters_s.l1_d   = lROSParamVehicleParamL1_d;
    lVehicleParameters_s.l2_d   = lROSParamVehicleParamL2_d;
    lVehicleParameters_s.swr_d  = lROSParamVehicleParamSWR_d;

    cPositionEstimation lPositionEstimation_cl( lROSParamDynamicTimeCalcEnabled_b,
                                                lROSParamLoopRateHz_i32,
                                                lROSParamVehicleType_s, 
                                                lVehicleParameters_s,
                                                lROSParamKinematicModelMaxSpeed_d);

    ROS_INFO_STREAM("ROS::OK  " << ros::ok());

    while (ros::ok())
    {
        visualization_msgs::Marker lROSEstAccuracyMarker_msg;
        geometry_msgs::PoseStamped lROSEstPoseCog_msg;
        geometry_msgs::PoseStamped lROSEstPoseBaselink_msg;
        std_msgs::Float32 lROSEstTravDistOdom_msg;
        std_msgs::Float32 lROSEstTravDistEstPos_msg;
        double lAccuracyScaleFactor_d = 10;

        struct timeval lTimeval_tv;
        gettimeofday(&lTimeval_tv, NULL);
        unsigned long long lCurrentTime_u64 = 
            (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
            (unsigned long long)(lTimeval_tv.tv_usec) / 1000;

        if ( (((lCurrentTime_u64 - gVehicleStatusMsgArriveTime_u64) < lROSParamMsgTimeout_d) && ((lCurrentTime_u64 - gIMUMsgArriveTime_u64) < lROSParamMsgTimeout_d)) && ((gPoseMsgArrived_b || lROSParamDoNotWaitForGnssMsgs_b) && gIMUMsgArrived_b && gVehicleStatusMsgArrived_b)) {
            sModelStates lCurrentModelStates_st;
            double lTmpRoll_d = 0;
            double lTmpPitch_d = 0;
            double lTmpYaw_d = 0;

            if ( (gROSCogPositionMsg_msg.pose.orientation.x != 0) || (gROSCogPositionMsg_msg.pose.orientation.y != 0) || (gROSCogPositionMsg_msg.pose.orientation.z != 0) || (gROSCogPositionMsg_msg.pose.orientation.w != 0) ) {
                tf::Quaternion lTmpQuternion_c(
                    gROSCogPositionMsg_msg.pose.orientation.x,
                    gROSCogPositionMsg_msg.pose.orientation.y,
                    gROSCogPositionMsg_msg.pose.orientation.z,
                    gROSCogPositionMsg_msg.pose.orientation.w);
                tf::Matrix3x3 lTmpMatrix(lTmpQuternion_c);

                lTmpMatrix.getRPY(lTmpRoll_d, lTmpPitch_d, lTmpYaw_d);
            }

            if ((lROSParamPoseTopic_s == "gps/duro/current_pose") && (lROSParamVehicleType_s == "SZEmission") && (lPositionEstimation_cl.getFiltMeasOri() != INVALID_ORIENTATION)) {
                lPositionEstimation_cl.setMeasuredValuesGNSS(gROSCogPositionMsg_msg.pose.position.x, gROSCogPositionMsg_msg.pose.position.y, gROSCogPositionMsg_msg.pose.position.z, lPositionEstimation_cl.getFiltMeasOri());
            } else {
                lPositionEstimation_cl.setMeasuredValuesGNSS(gROSCogPositionMsg_msg.pose.position.x, gROSCogPositionMsg_msg.pose.position.y, gROSCogPositionMsg_msg.pose.position.z, lTmpYaw_d);
            }
            if ((lROSParamImuTopic_s == "gps/duro/imu") || (lROSParamImuTopic_s == "imu/data")) {
                lPositionEstimation_cl.setMeasuredValuesIMU(gROSIMUMsg_msg.linear_acceleration.x, gROSIMUMsg_msg.linear_acceleration.y, gROSIMUMsg_msg.linear_acceleration.z, gROSIMUMsg_msg.angular_velocity.x, gROSIMUMsg_msg.angular_velocity.y, -1*gROSIMUMsg_msg.angular_velocity.z);
            } else {
                lPositionEstimation_cl.setMeasuredValuesIMU(gROSIMUMsg_msg.linear_acceleration.x, gROSIMUMsg_msg.linear_acceleration.y, gROSIMUMsg_msg.linear_acceleration.z, gROSIMUMsg_msg.angular_velocity.x, gROSIMUMsg_msg.angular_velocity.y, gROSIMUMsg_msg.angular_velocity.z);    
            }
            lPositionEstimation_cl.setMeasuredValuesVehicleState(gROSVehicleStatusMsg_msg.angle*1, gROSVehicleStatusMsg_msg.speed*1);
   
            lPositionEstimation_cl.iterateEstimation(lROSParamGnssSource_s,
                                                     lROSParamEstimationMethod_i32,
                                                     lROSParamVehicleType_s,
                                                     gDuroStatusMsgArrived_b,
                                                     gROSDuroStatusMsg_msg.data,
                                                     gNovatelStatusMsgArrived_b,
                                                     gROSNovatelStatusMsg_msg.position_type);
            lPositionEstimation_cl.getModelStates(&lCurrentModelStates_st);

            lAccuracyScaleFactor_d = lPositionEstimation_cl.getAccuracyScaleFactor();

            lROSEstPoseCog_msg.header.stamp = ros::Time::now();
            lROSEstPoseCog_msg.header.frame_id = "pose_cog";
            lROSEstPoseCog_msg.pose.position.x = lCurrentModelStates_st.positionX_d;
            lROSEstPoseCog_msg.pose.position.y = lCurrentModelStates_st.positionY_d;
            lROSEstPoseCog_msg.pose.position.z = 0;
            lROSEstPoseCog_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, lCurrentModelStates_st.yawAngle_d);
        }
        else {
            lROSEstPoseCog_msg.header.stamp = ros::Time::now();
            lROSEstPoseCog_msg.header.frame_id = "pose_cog";
            lROSEstPoseCog_msg.pose.position.x = 0;
            lROSEstPoseCog_msg.pose.position.y = 0;
            lROSEstPoseCog_msg.pose.position.z = 0;
            lROSEstPoseCog_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        }

        lROSPubEstimatedPoseCog_cl.publish(lROSEstPoseCog_msg);

        lROSEstTravDistOdom_msg.data = lPositionEstimation_cl.getTravDistanceOdom();
        lROSEstTravDistEstPos_msg.data = lPositionEstimation_cl.getTravDistanceEstPos();
        lROSPubEstimatedTraveledDistanceOdom_cl.publish(lROSEstTravDistOdom_msg);
        lROSPubEstimatedTraveledDistanceEstPos_cl.publish(lROSEstTravDistEstPos_msg);
        
        
        tf2_ros::Buffer lBuffer_cl;

        lROSEstPoseBaselink_msg.header.frame_id = "pose_base_link";
        lROSEstPoseBaselink_msg.header.stamp = ros::Time::now();
        lROSEstPoseBaselink_msg.pose.position.x = -lPositionEstimation_cl.getCogDistanceFromBaselinkX();
        lROSEstPoseBaselink_msg.pose.position.y =  lPositionEstimation_cl.getCogDistanceFromBaselinkY();
        lROSEstPoseBaselink_msg.pose.position.z =  lPositionEstimation_cl.getCogDistanceFromBaselinkZ();
        lROSEstPoseBaselink_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        
        geometry_msgs::TransformStamped lEstPose_tr;
        lEstPose_tr.header.frame_id = lROSEstPoseCog_msg.header.frame_id;
        lEstPose_tr.child_frame_id = lROSEstPoseBaselink_msg.header.frame_id;
        lEstPose_tr.transform.translation.x = lROSEstPoseCog_msg.pose.position.x;
        lEstPose_tr.transform.translation.y = lROSEstPoseCog_msg.pose.position.y;
        lEstPose_tr.transform.translation.z = lROSEstPoseCog_msg.pose.position.z;
        lEstPose_tr.transform.rotation = lROSEstPoseCog_msg.pose.orientation;
        lBuffer_cl.setTransform(lEstPose_tr, "default_authority", true);
        
        geometry_msgs::TransformStamped lTsLookup_cl;
        lTsLookup_cl = lBuffer_cl.lookupTransform( lROSEstPoseCog_msg.header.frame_id, lROSEstPoseBaselink_msg.header.frame_id, ros::Time(0));

        tf2::doTransform(lROSEstPoseBaselink_msg, lROSEstPoseBaselink_msg, lTsLookup_cl);

        lROSEstPoseBaselink_msg.header.frame_id = "pose_base_link";
        lROSEstPoseBaselink_msg.header.stamp = ros::Time::now();

        lROSPubEstimatedPoseBaselink_cl.publish(lROSEstPoseBaselink_msg);

        // Broadcast tf2 transform from here

        static tf2_ros::TransformBroadcaster br;

        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = lROSEstPoseBaselink_msg.pose.position.x;
        transformStamped.transform.translation.y = lROSEstPoseBaselink_msg.pose.position.y;
        transformStamped.transform.translation.z = lROSEstPoseBaselink_msg.pose.position.z;
        tf2::Quaternion q;
        transformStamped.transform.rotation.x = lROSEstPoseBaselink_msg.pose.orientation.x;
        transformStamped.transform.rotation.y = lROSEstPoseBaselink_msg.pose.orientation.y;
        transformStamped.transform.rotation.z = lROSEstPoseBaselink_msg.pose.orientation.z;
        transformStamped.transform.rotation.w = lROSEstPoseBaselink_msg.pose.orientation.w;

        br.sendTransform(transformStamped);

        lROSEstAccuracyMarker_msg.header.frame_id = "base_link";
        lROSEstAccuracyMarker_msg.header.stamp = ros::Time();
        lROSEstAccuracyMarker_msg.ns = lROSParamEstimatedPoseBaselinkTopic_s;
        lROSEstAccuracyMarker_msg.id = 0;
        lROSEstAccuracyMarker_msg.type = visualization_msgs::Marker::SPHERE;
        lROSEstAccuracyMarker_msg.action = visualization_msgs::Marker::ADD;
        lROSEstAccuracyMarker_msg.pose.position.x = 0.0;
        lROSEstAccuracyMarker_msg.pose.position.y = 0.0;
        lROSEstAccuracyMarker_msg.pose.position.z = 0.6;
        /*
        lROSEstAccuracyMarker_msg.pose.position.x = lROSEstPoseCog_msg.pose.position.x;
        lROSEstAccuracyMarker_msg.pose.position.y = lROSEstPoseCog_msg.pose.position.y;
        lROSEstAccuracyMarker_msg.pose.position.z = lROSEstPoseCog_msg.pose.position.z;
        */
        lROSEstAccuracyMarker_msg.pose.orientation.x = 0.0;
        lROSEstAccuracyMarker_msg.pose.orientation.y = 0.0;
        lROSEstAccuracyMarker_msg.pose.orientation.z = 0.0;
        lROSEstAccuracyMarker_msg.pose.orientation.w = 1.0;
        if (gNavSatFixMsgArrived_b) {
            lROSEstAccuracyMarker_msg.scale.x = lAccuracyScaleFactor_d * (sqrt(gROSNavSatFixMsg_msg.position_covariance[0] * gROSNavSatFixMsg_msg.position_covariance[4]))/2;
            lROSEstAccuracyMarker_msg.scale.y = lAccuracyScaleFactor_d * (sqrt(gROSNavSatFixMsg_msg.position_covariance[0] * gROSNavSatFixMsg_msg.position_covariance[4]))/2;
            lROSEstAccuracyMarker_msg.scale.z = lAccuracyScaleFactor_d * (sqrt(gROSNavSatFixMsg_msg.position_covariance[0] * gROSNavSatFixMsg_msg.position_covariance[4]))/2;            
        } else {
            lROSEstAccuracyMarker_msg.scale.x = lAccuracyScaleFactor_d * 10;
            lROSEstAccuracyMarker_msg.scale.y = lAccuracyScaleFactor_d * 10;
            lROSEstAccuracyMarker_msg.scale.z = lAccuracyScaleFactor_d * 10;
        }
        lROSEstAccuracyMarker_msg.color.a = 0.3; // Don't forget to set the alpha!
        lROSEstAccuracyMarker_msg.color.r = 0.1;
        lROSEstAccuracyMarker_msg.color.g = 0.8;
        lROSEstAccuracyMarker_msg.color.b = 0.4;
        //only if using a MESH_RESOURCE marker type:
        //lROSEstAccuracyMarker_msg.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        lROSPubAccuracyMarker_cl.publish( lROSEstAccuracyMarker_msg );

        ros::spinOnce();
        lROSLoopRate_cl.sleep();
    }
    return 0;
}
