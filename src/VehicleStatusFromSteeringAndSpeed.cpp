// ROS 2 node to publish vehicle status from steering and speed
// speed topic eg: /nissan/vehicle_speed        std_msgs/msg/Float32
// steering topic eg: /nissan/vehicle_steering  std_msgs/msg/Float32 
// output topic eg: /nissan/vehicle_status      geometry_msgs/msg/TwistStamped

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("vehicle_status_converter")
    {
        this->declare_parameter("speed_topic", "/nissan/vehicle_speed");
        this->declare_parameter("steer_topic", "/nissan/vehicle_steering");
        this->declare_parameter("status_topic", "/nissan/vehicle_status");

        this->get_parameter("speed_topic", speed_topic_);
        this->get_parameter("steer_topic", steer_topic_);
        this->get_parameter("status_topic", status_topic_);

        sub_speed_ = this->create_subscription<std_msgs::msg::Float32>(speed_topic_, 10, std::bind(&MinimalSubscriber::speed_callback, this, _1)); 
        sub_steer_ = this->create_subscription<std_msgs::msg::Float32>(steer_topic_, 10, std::bind(&MinimalSubscriber::steer_callback, this, _1));
        pub_vehicle_status_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(status_topic_, 10);
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed: " << speed_topic_ << " | " << steer_topic_ << " | Publishing: " << status_topic_); 
    }

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        vehicle_status_.header.stamp = this->now();
        vehicle_status_.twist.linear.x = msg->data;
        pub_vehicle_status_->publish(vehicle_status_);
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Subscribed to speed | " << msg->data); 
    }
    void steer_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        vehicle_status_.header.stamp = this->now();
        vehicle_status_.twist.angular.z = msg->data;
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Subscribed to steering | " << msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steer_, sub_speed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_vehicle_status_;
    geometry_msgs::msg::TwistStamped vehicle_status_;
    std::string speed_topic_, steer_topic_, status_topic_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}