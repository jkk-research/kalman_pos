#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "autoware_msgs/VehicleStatus.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>

geometry_msgs::PoseStamped orig_pose_;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  orig_pose_ = *msg;
  //ROS_INFO_STREAM("x: " << msg->pose.position.x << " y: " << msg->pose.position.y);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO_STREAM("acc x: " << msg->linear_acceleration.x << " y: " << msg->linear_acceleration.y);
}

void vehicleCallback(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
  //ROS_INFO_STREAM("speed: " << msg->speed << " angle: " << msg->angle);
}



int main(int argc, char **argv)
{
    std::string pose_topic, imu_topic, estimated_pose, estimated_debug_pose;
    bool debug;
    ros::init(argc, argv, "kalman_pos_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("pose_topic", pose_topic, "gps/nova/current_pose");
    n_private.param<std::string>("imu_topic", imu_topic, "gps/nova/imu");
    n_private.param<std::string>("est_topic", estimated_pose, "estimated_pose");
    n_private.param<std::string>("est_debug_topic", estimated_debug_pose, "estimated_debug_pose");
    n_private.param<bool>("debug", debug, false);
    ROS_INFO_STREAM("kalman_pos_node started | " << pose_topic << " | debug: " << debug);
    
    ros::Publisher est_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_pose, 1000);
    ros::Publisher est_debug_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_debug_pose, 1000);
    ros::Subscriber sub_pose = n.subscribe(pose_topic, 1000, poseCallback);
    ros::Subscriber sub_imu = n.subscribe(imu_topic, 1000, imuCallback);
    ros::Subscriber sub_vehicle = n.subscribe("vehicle_status", 1000, vehicleCallback);
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok())
    {
      geometry_msgs::PoseStamped est_pose_msg;
      geometry_msgs::PoseStamped est_debug_pose_msg;
      est_pose_msg.header.stamp = ros::Time::now();
      est_pose_msg.header.frame_id = "map";
      est_pose_msg.pose.position = orig_pose_.pose.position;
      est_pose_msg.pose.position.x += 1000000;
      //ROS_INFO_STREAM("x: " << est_pose_msg.pose.position.x << " y: " << est_pose_msg.pose.position.y);
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