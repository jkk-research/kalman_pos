// no autoware version

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO_STREAM("x: " << msg->pose.position.x << " y: " << msg->pose.position.y);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO_STREAM("acc x: " << msg->linear_acceleration.x << " y: " << msg->linear_acceleration.y);
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
    ros::Publisher est_debug_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_pose, 1000);
    ros::Subscriber sub_pose = n.subscribe(pose_topic, 1000, poseCallback);
    ros::Subscriber sub_imu = n.subscribe(imu_topic, 1000, imuCallback);
    ros::Rate loop_rate(40); // 40 Hz
    while (ros::ok())
    {
      geometry_msgs::PoseStamped est_pose_msg;
      geometry_msgs::PoseStamped est_debug_pose_msg;
      est_pub.publish(est_pose_msg);
      if (debug){
        est_debug_pub.publish(est_debug_pose_msg);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }


    return 0;
}