#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <autoware_msgs/VehicleStatus.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>
#include <novatel_msgs/INSPVAX.h>

//#include "geometry_msgs/Vector3.h"
//#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"

#include "CombinedVehicleModel.h"

geometry_msgs::PoseStamped orig_pose_;

geometry_msgs::PoseStamped gPositionMsg;
sensor_msgs::Imu gIMUMsg;
autoware_msgs::VehicleStatus gVehicleStatusMsg;
novatel_msgs::INSPVAX gNovatelStatus;

bool gPoseMsgArrived_b = false;
bool gIMUMsgArrived_b = false;
bool gVehicleStatusMsgArrived_b = false;
bool gNovatelStatusMsgArrived_b = false;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gPoseMsgArrived_b = true;
    orig_pose_ = *msg;
    gPositionMsg = *msg;
    //ROS_INFO_STREAM("x: " << msg->pose.position.x << " y: " << msg->pose.position.y);
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

void novatelStatusCallback(const novatel_msgs::INSPVAX::ConstPtr& msg)
{
    gNovatelStatusMsgArrived_b = true;
    gNovatelStatus = *msg;
}


int main(int argc, char **argv)
{
    std::string pose_topic, imu_topic, estimated_pose, estimated_debug_pose;
    bool debug;
    int loop_rate_hz;
    int estimation_method;
    cCombinedVehicleModel lCombinedVehicleModel;
    bool lFirstIteration = true;

    ros::init(argc, argv, "kalman_pos_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("pose_topic", pose_topic, "gps/nova/current_pose");
    n_private.param<std::string>("imu_topic", imu_topic, "gps/nova/imu");
    n_private.param<std::string>("est_topic", estimated_pose, "estimated_pose");
    n_private.param<std::string>("est_debug_topic", estimated_debug_pose, "estimated_debug_pose");
    n_private.param<bool>("debug", debug, false);
    n_private.param<int>("loop_rate_hz", loop_rate_hz, 10);
    n_private.param<int>("estimation_method", estimation_method, 0);
    ROS_INFO_STREAM("kalman_pos_node started | " << pose_topic << " | debug: " << debug);
    
    ros::Publisher est_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_pose, 1000);
    ros::Publisher est_debug_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_debug_pose, 1000);
    ros::Subscriber sub_pose = n.subscribe(pose_topic, 1000, poseCallback);
    ros::Subscriber sub_imu = n.subscribe(imu_topic, 1000, imuCallback);
    ros::Subscriber sub_vehicle = n.subscribe("vehicle_status", 1000, vehicleCallback);
    ros::Subscriber sub_inspvax = n.subscribe("gps/nova/inspvax", 1000, novatelStatusCallback);
    ros::Rate loop_rate(loop_rate_hz); // 10 Hz

    gPoseMsgArrived_b = false;
    gIMUMsgArrived_b = false;
    gVehicleStatusMsgArrived_b = false;
    gNovatelStatusMsgArrived_b = false;

    lCombinedVehicleModel.initVehicleParameters();
    lCombinedVehicleModel.initEKFMatrices();

    while (ros::ok())
    {
      geometry_msgs::PoseStamped est_pose_msg;
      geometry_msgs::PoseStamped est_debug_pose_msg;
      if (gPoseMsgArrived_b && gIMUMsgArrived_b && gVehicleStatusMsgArrived_b && gNovatelStatusMsgArrived_b) {
          
        sModelStates lCurrentModelStates_s;

        eEstimationMode lEstimationMode_e = eEstimationMode::ekf;
        eGNSSState lGNSSState_e = eGNSSState::off;

        double lTmpRoll_d;
        double lTmpPitch_d;
        double lTmpYaw_d;

        tf::Quaternion lTmpQuternion_c(
            gPositionMsg.pose.orientation.x,
            gPositionMsg.pose.orientation.y,
            gPositionMsg.pose.orientation.z,
            gPositionMsg.pose.orientation.w);
        tf::Matrix3x3 lTmpMatrix(lTmpQuternion_c);

        lTmpMatrix.getRPY(lTmpRoll_d, lTmpPitch_d, lTmpYaw_d);

        //lCombinedVehicleModel.setPrevMeasuredValues();
        lCombinedVehicleModel.setMeasuredValuesGNSS(gPositionMsg.pose.position.x, gPositionMsg.pose.position.y, gPositionMsg.pose.position.z, lTmpYaw_d);
        lCombinedVehicleModel.setMeasuredValuesIMU(gIMUMsg.linear_acceleration.x, gIMUMsg.linear_acceleration.y, gIMUMsg.linear_acceleration.z, gIMUMsg.angular_velocity.x, gIMUMsg.angular_velocity.y, gIMUMsg.angular_velocity.z);
        lCombinedVehicleModel.setMeasuredValuesVehicleState(gVehicleStatusMsg.angle, gVehicleStatusMsg.speed);
              
        if (lFirstIteration) {
            lCombinedVehicleModel.setPrevEKFMatrices();
            lCombinedVehicleModel.setPrevMeasuredValues();
            lCombinedVehicleModel.setModelStates(0, gIMUMsg.angular_velocity.z, lTmpYaw_d, gIMUMsg.linear_acceleration.y, gPositionMsg.pose.position.x, gPositionMsg.pose.position.y, gVehicleStatusMsg.speed, 0);
            lFirstIteration = false;
        }

        /*
          uint32 position_type
          uint32 POSITION_TYPE_NONE=0
          uint32 POSITION_TYPE_SBAS=52
          uint32 POSITION_TYPE_PSEUDORANGE_SINGLE_POINT=53
          uint32 POSITION_TYPE_PSEUDORANGE_DIFFERENTIAL=54
          uint32 POSITION_TYPE_RTK_FLOAT=55
          uint32 POSITION_TYPE_RTK_FIXED=56
          uint32 POSITION_TYPE_OMNISTAR=57
          uint32 POSITION_TYPE_OMNISTAR_HP=58
          uint32 POSITION_TYPE_OMNISTAR_XP=59
          uint32 POSITION_TYPE_PPP_CONVERGING=73
          uint32 POSITION_TYPE_PPP=74
        */
        switch (gNovatelStatus.position_type) {
          case 0: //POSITION_TYPE_NONE
              lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
              lGNSSState_e = eGNSSState::off;
            break;
          case 52: //POSITION_TYPE_SBAS
              lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
              lGNSSState_e = eGNSSState::SBAS;
            break;
          case 53: //POSITION_TYPE_PSEUDORANGE_SINGLE_POINT
              lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
              lGNSSState_e = eGNSSState::pseudorange;
            break;
          case 54: //POSITION_TYPE_PSEUDORANGE_DIFFERENTIAL
              lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
              lGNSSState_e = eGNSSState::pseudorange;
            break;
          case 55: //POSITION_TYPE_RTK_FLOAT
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_float;
            break;
          case 56: //POSITION_TYPE_RTK_FIXED
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_fixed;
            break;
          case 57: //POSITION_TYPE_OMNISTAR
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_fixed;
            break;
          case 58: //POSITION_TYPE_OMNISTAR_HP
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_fixed;
            break;
          case 59: //POSITION_TYPE_OMNISTAR_XP
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_fixed;
            break;
          case 73: //POSITION_TYPE_PPP_CONVERGING
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_fixed;
            break;
          case 74: //POSITION_TYPE_PPP
              lEstimationMode_e = eEstimationMode::ekf;
              lGNSSState_e = eGNSSState::rtk_fixed;
            break;
          default:
              lEstimationMode_e = eEstimationMode::ekf_ekf_wognss;
              lGNSSState_e = eGNSSState::off;
            break;
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

      if (debug){
        est_debug_pose_msg.header.stamp = ros::Time::now();
        est_debug_pub.publish(est_debug_pose_msg);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }


    return 0;
}