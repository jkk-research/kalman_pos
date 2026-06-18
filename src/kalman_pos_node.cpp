#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
// #include <novatel_gps_msgs/msg/inspvax.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"

#include <sstream>
#include <time.h>
#include <sys/time.h>

#include "PositionEstimation.h"

using namespace std;

class KalmanPosNode : public rclcpp::Node 
{
    public:
        KalmanPosNode() : Node("kalman_pos_node") {

            RCLCPP_INFO_ONCE(this->get_logger(), "Init Variables");
            iDrivingMode_i32 = 0;
            iPrevDrivingMode_i32 = 0;
            iGNSSPoseMsgArrived_b = false;
            iSLAMPoseMsgArrived_b = false;
            iGNSSCovarianceMsgArrived_b = false;
            iSLAMCovarianceMsgArrived_b = false;
            iIMUMsgArrived_b = false;
            iVehicleStatusMsgArrived_b = false;
            iIMUMsgArriveTime_u64 = 0;
            iVehicleStatusMsgArriveTime_u64 = 0;
            iOriEstimationEnabled_b = false;
            iROSParamInvertYawRate_b = false;

            RCLCPP_INFO_ONCE(this->get_logger(), "Init Parameters");
            this->declare_parameter<std::string>("gnss_pose_topic", "gps/duro/current_pose");
            this->declare_parameter<std::string>("slam_pose_topic", "gps/duro/current_pose");
            this->declare_parameter<std::string>("vehicle_status_topic", "vehicle_status");
            this->declare_parameter<std::string>("gnss_covariance_topic", "gps/duro/fix");
            this->declare_parameter<std::string>("slam_covariance_topic", "gps/duro/fix");
            this->declare_parameter<std::string>("imu_topic", "imu/data");
            this->declare_parameter<std::string>("est_cog_topic", "estimated_pose_cog");
            this->declare_parameter<std::string>("est_baselink_topic", "estimated_pose_baselink");
            this->declare_parameter<std::string>("est_accuracy_topic",  "estimation_accuracy");
            this->declare_parameter<std::string>("est_trav_distance_odom_topic", "distance");
            this->declare_parameter<std::string>("est_trav_distance_est_pos_topic", "estimated_trav_dist_est_pos");
            this->declare_parameter<int>("loop_rate_hz", 60);
            this->declare_parameter<bool>("gnss_available", false);
            this->declare_parameter<bool>("slam_available", false);
            this->declare_parameter<float>("gnss_accuracy_limit", 10.0);
            this->declare_parameter<float>("slam_accuracy_limit", 10.0);
            this->declare_parameter<float>("gnss_default_covariance", 15.0);
            this->declare_parameter<float>("slam_default_covariance", 15.0); 
            this->declare_parameter<bool>("dynamic_time_calc",  true);
            this->declare_parameter<bool>("do_not_wait_for_pos_msgs", true);
            this->declare_parameter<double>("kinematic_model_max_speed", 0.3);
            this->declare_parameter<bool>("use_raw_model", false);
            this->declare_parameter<bool>("orientation_est_enabled", false);
            this->declare_parameter<bool>("invert_yaw_rate", false);
            this->declare_parameter<double>("msg_timeout", 2000);
            this->declare_parameter<double>("vehicle_param_c1", 3000);
            this->declare_parameter<double>("vehicle_param_c2", 3000);
            this->declare_parameter<double>("vehicle_param_m", 180);
            this->declare_parameter<double>("vehicle_param_jz", 270);
            this->declare_parameter<double>("vehicle_param_l1", 0.624);
            this->declare_parameter<double>("vehicle_param_l2", 0.676);
            this->declare_parameter<double>("vehicle_param_swr", 1); 
            this->declare_parameter<std::string>("autonomous_mode_topic", "myrio_state");

            RCLCPP_INFO_ONCE(this->get_logger(), "Get Parameters");
            this->get_parameter("gnss_pose_topic", iROSParamGNSSPoseTopic_s);
            this->get_parameter("slam_pose_topic", iROSParamSLAMPoseTopic_s);
            this->get_parameter("vehicle_status_topic", iROSParamVehicleStatusTopic_s);
            this->get_parameter("imu_topic", iROSParamImuTopic_s);
            this->get_parameter("est_cog_topic", iROSParamEstimatedPoseCogTopic_s); 
            this->get_parameter("est_baselink_topic", iROSParamEstimatedPoseBaselinkTopic_s);
            this->get_parameter("est_trav_distance_odom_topic", iROSParamEstimatedTravDistOdom_s);
            this->get_parameter("est_trav_distance_est_pos_topic", iROSParamEstimatedTravDistEstPos_s);
            this->get_parameter("est_accuracy_topic", iROSParamEstimationAccuracyTopic_s);
            this->get_parameter("gnss_covariance_topic", iROSParamGNSSCovarianceTopic_s);
            this->get_parameter("slam_covariance_topic", iROSParamSLAMCovarianceTopic_s);
            this->get_parameter("vehicle_status_topic", iROSParamVehicleStatusTopic_s);
            this->get_parameter("dynamic_time_calc", iROSParamDynamicTimeCalcEnabled_b); 
            this->get_parameter("do_not_wait_for_pos_msgs", iROSParamDoNotWaitForPosMsgs_b);
            this->get_parameter("loop_rate_hz", iROSParamLoopRateHz_i32);
            this->get_parameter("gnss_available", iROSParamGNSSAvailable_b);
            this->get_parameter("slam_available", iROSParamSLAMAvailable_b);
            this->get_parameter("gnss_accuracy_limit", iROSParamGNSSAccuracyLimit_d);
            this->get_parameter("slam_accuracy_limit", iROSParamSLAMAccuracyLimit_d);
            this->get_parameter("gnss_default_covariance", iROSParamGNSSDefaultCovariance_d);
            this->get_parameter("slam_default_covariance", iROSParamSLAMDefaultCovariance_d);
            this->get_parameter("kinematic_model_max_speed", iROSParamKinematicModelMaxSpeed_d);
            this->get_parameter("use_raw_model", iROSParamUseRawModel_b);
            this->get_parameter("orientation_est_enabled", iOriEstimationEnabled_b);
            this->get_parameter("msg_timeout", iROSParamMsgTimeout_d);
            this->get_parameter("invert_yaw_rate", iROSParamInvertYawRate_b);
            this->get_parameter("vehicle_param_c1", iROSParamVehicleParamC1_d);
            this->get_parameter("vehicle_param_c2", iROSParamVehicleParamC2_d);
            this->get_parameter("vehicle_param_m", iROSParamVehicleParamM_d);
            this->get_parameter("vehicle_param_jz", iROSParamVehicleParamJz_d);
            this->get_parameter("vehicle_param_l1", iROSParamVehicleParamL1_d);
            this->get_parameter("vehicle_param_l2", iROSParamVehicleParamL2_d);
            this->get_parameter("vehicle_param_swr", iROSParamVehicleParamSWR_d);
            this->get_parameter("autonomous_mode_topic", iROSParamAutonomousModeTopic_s);

            RCLCPP_INFO_ONCE(this->get_logger(), "Create Subscriptions");
            iROSSubGNSSPose_cl = this->create_subscription<geometry_msgs::msg::PoseStamped>(iROSParamGNSSPoseTopic_s, 1000, std::bind(&KalmanPosNode::gnssPoseCallback, this, std::placeholders::_1));
            iROSSubSLAMPose_cl = this->create_subscription<geometry_msgs::msg::PoseStamped>(iROSParamSLAMPoseTopic_s, 1000, std::bind(&KalmanPosNode::slamPoseCallback, this, std::placeholders::_1));
            iROSSubVehicleStatus_cl = this->create_subscription<geometry_msgs::msg::TwistStamped>(iROSParamVehicleStatusTopic_s, 1000, std::bind(&KalmanPosNode::vehicleCallback, this, std::placeholders::_1));
            iROSSubGNSSCovariance_cl = this->create_subscription<sensor_msgs::msg::NavSatFix>(iROSParamGNSSCovarianceTopic_s, 1000, std::bind(&KalmanPosNode::gnssCovarianceCallback, this, std::placeholders::_1));
            iROSSubSLAMCovariance_cl = this->create_subscription<sensor_msgs::msg::NavSatFix>(iROSParamSLAMCovarianceTopic_s, 1000, std::bind(&KalmanPosNode::slamCovarianceCallback, this, std::placeholders::_1));
            iROSSubIMU_cl = this->create_subscription<sensor_msgs::msg::Imu>(iROSParamImuTopic_s, 1000, std::bind(&KalmanPosNode::imuCallback, this, std::placeholders::_1));
            iROSSubAutonomousMode_cl = this->create_subscription<std_msgs::msg::Bool>(iROSParamAutonomousModeTopic_s, 1000, std::bind(&KalmanPosNode::autonomousModeCallback, this, std::placeholders::_1)); 

            RCLCPP_INFO_ONCE(this->get_logger(), "Create Publisher");
            iROSPubEstimatedPoseCog_cl = this->create_publisher<geometry_msgs::msg::PoseStamped>(iROSParamEstimatedPoseCogTopic_s, 1000);
            iROSPubEstimatedPoseBaselink_cl = this->create_publisher<geometry_msgs::msg::PoseStamped>(iROSParamEstimatedPoseBaselinkTopic_s, 1000);
            iROSPubAccuracyMarker_cl = this->create_publisher<visualization_msgs::msg::Marker>(iROSParamEstimationAccuracyTopic_s, 1000);
            iROSPubEstimatedTraveledDistanceOdom_cl = this->create_publisher<std_msgs::msg::Float32>(iROSParamEstimatedTravDistOdom_s, 1000);
            iROSPubEstimatedTraveledDistanceEstPos_cl = this->create_publisher<std_msgs::msg::Float32>(iROSParamEstimatedTravDistEstPos_s, 1000);
            
            RCLCPP_INFO_ONCE(this->get_logger(), "Create Transform Broadcaster");
            iEstPosBaselinkTransformBroadcaster_cl = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            RCLCPP_INFO_ONCE(this->get_logger(), "Set Parameter Callback");
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&KalmanPosNode::parametersCallback, this, std::placeholders::_1));

            RCLCPP_INFO_ONCE(this->get_logger(), "Set Vehicle Parameters");
            iVehicleParameters_s.iC1_d   = iROSParamVehicleParamC1_d;
            iVehicleParameters_s.iC2_d   = iROSParamVehicleParamC2_d;
            iVehicleParameters_s.iM_d    = iROSParamVehicleParamM_d;
            iVehicleParameters_s.iJz_d   = iROSParamVehicleParamJz_d;
            iVehicleParameters_s.iL1_d   = iROSParamVehicleParamL1_d;
            iVehicleParameters_s.iL2_d   = iROSParamVehicleParamL2_d;
            iVehicleParameters_s.iSwr_d  = iROSParamVehicleParamSWR_d;

            RCLCPP_INFO_ONCE(this->get_logger(), "Init Estimation");
            iPositionEstimation_cl.initEstimation(  iROSParamDynamicTimeCalcEnabled_b,
                                                    iROSParamLoopRateHz_i32,
                                                    iVehicleParameters_s,
                                                    iROSParamKinematicModelMaxSpeed_d,
                                                    iOriEstimationEnabled_b);

            RCLCPP_INFO_ONCE(this->get_logger(), "Set Timer and Timer Callback");
            double iROSParamLoopRateHz_dbl = static_cast<double>(iROSParamLoopRateHz_i32);
            int millisec = (1/iROSParamLoopRateHz_dbl) * 1000;
            
            timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(millisec),
                    std::bind(&KalmanPosNode::timerCallback, this));
            

            // RCLCPP_INFO_STREAM(this->get_logger(), "Loop rate: " << (1/iROSParamLoopRateHz_i32) * 1000 << " ms"); // NOK
            RCLCPP_INFO_STREAM(this->get_logger(), "Loop rate: " << millisec << " ms");

            RCLCPP_INFO_ONCE(this->get_logger(), "Finalized Parameters:\n\tGNSS Pose Topic: %s\n\tSLAM Pose Topic: %s\n\tVehicle Status Topic: %s\n\tNavSatFix Topic: %s\n\tIMU Topic: %s\n\tEst. CoG. Pos. Topic: %s\n\tEst. Baselink Pos. Topic: %s\n\tEst. Accuracy Topic: %s\n\tLoop Rate [Hz]: %d\n\tGNSS Available: %d\n\tSLAM Available: %d\n\tDyn. Time Calc. Enabled: %d\n\tDo Not Wait For GNSS Msgs.: %d\n\tKinematic Model Max. Speed: %f\n\tMsg. Timeout: %f\n\tInvert Yaw Rate: %d\n\tEst. Trav. Dist. (Odom) Topic: %s\n\tEst. Trav. Dist. (Pos.) Topic: %s\n\tV.P. C1: %f\n\tV.P. C2: %f\n\tV.P. m: %f\n\tV.P. Jz: %f\n\tV.P. L1: %f\n\tV.P. L2: %f\n\tV.P. SWR: %f\n\tGNSS Accuracy Limit: %f\n\tSLAM Accuracy Limit: %f\n\tUse Raw Model Only: %d\n\tOri. Est. Enabled: %d\n\tAutonomous Mode Topic: %s", 
                                                    iROSParamGNSSPoseTopic_s.c_str(), iROSParamSLAMPoseTopic_s.c_str(), iROSParamVehicleStatusTopic_s.c_str(), 
                                                    iROSParamGNSSCovarianceTopic_s.c_str(), iROSParamImuTopic_s.c_str(), iROSParamEstimatedPoseCogTopic_s.c_str(),  
                                                    iROSParamEstimatedPoseBaselinkTopic_s.c_str(), iROSParamEstimationAccuracyTopic_s.c_str(),
                                                    iROSParamLoopRateHz_i32, iROSParamGNSSAvailable_b, iROSParamSLAMAvailable_b, iROSParamDynamicTimeCalcEnabled_b, 
                                                    iROSParamDoNotWaitForPosMsgs_b, iROSParamKinematicModelMaxSpeed_d, iROSParamMsgTimeout_d, iROSParamInvertYawRate_b,
                                                    iROSParamEstimatedTravDistOdom_s.c_str(), iROSParamEstimatedTravDistEstPos_s.c_str(), iROSParamVehicleParamC1_d, 
                                                    iROSParamVehicleParamC2_d, iROSParamVehicleParamM_d, iROSParamVehicleParamJz_d,
                                                    iROSParamVehicleParamL1_d, iROSParamVehicleParamL2_d, iROSParamVehicleParamSWR_d,
                                                    iROSParamGNSSAccuracyLimit_d, iROSParamSLAMAccuracyLimit_d, iROSParamUseRawModel_b, iOriEstimationEnabled_b,
                                                    iROSParamAutonomousModeTopic_s.c_str());

            RCLCPP_INFO_ONCE(this->get_logger(), "Finalize Initialization");
        }

    public:
        int iROSParamLoopRateHz_i32;
    private:
        // Declare ROS parameter variables
        std::string iROSParamGNSSPoseTopic_s;
        std::string iROSParamAutonomousModeTopic_s;
        std::string iROSParamSLAMPoseTopic_s;
        std::string iROSParamImuTopic_s;
        std::string iROSParamEstimatedPoseCogTopic_s; 
        std::string iROSParamEstimatedPoseBaselinkTopic_s;
        std::string iROSParamEstimatedTravDistOdom_s;
        std::string iROSParamEstimatedTravDistEstPos_s;
        std::string iROSParamEstimationAccuracyTopic_s;
        std::string iROSParamGNSSCovarianceTopic_s;
        std::string iROSParamSLAMCovarianceTopic_s;
        std::string iROSParamVehicleStatusTopic_s;
        bool iROSParamDynamicTimeCalcEnabled_b; 
        bool iROSParamDoNotWaitForPosMsgs_b;
        bool iROSParamGNSSAvailable_b;
        bool iROSParamSLAMAvailable_b;
        bool iROSParamUseRawModel_b;
        double iROSParamGNSSAccuracyLimit_d;
        double iROSParamSLAMAccuracyLimit_d;
        double iROSParamGNSSDefaultCovariance_d;
        double iROSParamSLAMDefaultCovariance_d;
        double iROSParamKinematicModelMaxSpeed_d;
        double iROSParamMsgTimeout_d;
        double iROSParamVehicleParamC1_d;
        double iROSParamVehicleParamC2_d;
        double iROSParamVehicleParamM_d;
        double iROSParamVehicleParamJz_d;
        double iROSParamVehicleParamL1_d;
        double iROSParamVehicleParamL2_d;
        double iROSParamVehicleParamSWR_d;
        bool iROSParamInvertYawRate_b;

        sVehicleParameters iVehicleParameters_s;
        cPositionEstimation iPositionEstimation_cl;

        int iDrivingMode_i32;
        int iPrevDrivingMode_i32;
        bool iOriEstimationEnabled_b;
        bool iGNSSPoseMsgArrived_b;
        bool iSLAMPoseMsgArrived_b;
        bool iGNSSCovarianceMsgArrived_b;
        bool iSLAMCovarianceMsgArrived_b;
        bool iIMUMsgArrived_b;
        bool iVehicleStatusMsgArrived_b;
        uint64_t iIMUMsgArriveTime_u64;
        uint64_t iVehicleStatusMsgArriveTime_u64;
        
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr iROSPubAccuracyMarker_cl;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr iROSPubEstimatedPoseCog_cl;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr iROSPubEstimatedPoseBaselink_cl;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iROSPubEstimatedTraveledDistanceOdom_cl;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iROSPubEstimatedTraveledDistanceEstPos_cl;
            
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr iROSSubGNSSPose_cl;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr iROSSubSLAMPose_cl;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr iROSSubGNSSCovariance_cl;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr iROSSubSLAMCovariance_cl;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr iROSSubIMU_cl;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr iROSSubVehicleStatus_cl;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr iROSSubAutonomousMode_cl;

        std::unique_ptr<tf2_ros::TransformBroadcaster> iEstPosBaselinkTransformBroadcaster_cl;

        geometry_msgs::msg::PoseStamped iROSGNSSPositionMsg_msg;
        geometry_msgs::msg::PoseStamped iROSSLAMPositionMsg_msg;
        geometry_msgs::msg::PoseStamped iROSGNSSCogPositionMsg_msg;
        geometry_msgs::msg::PoseStamped iROSSLAMCogPositionMsg_msg;
        sensor_msgs::msg::NavSatFix iROSGNSSCovarianceMsg_msg;
        sensor_msgs::msg::NavSatFix iROSSLAMCovarianceMsg_msg;
        sensor_msgs::msg::Imu iROSIMUMsg_msg;
        geometry_msgs::msg::TwistStamped iROSVehicleStatusMsg_msg;
        std_msgs::msg::Bool iROSAutonomousModeMsg_msg;

        void gnssPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "GNSS Pose Callback");
            iGNSSPoseMsgArrived_b = true;
            iROSGNSSPositionMsg_msg = *pMsg_msgp;
            iROSGNSSCogPositionMsg_msg = iROSGNSSPositionMsg_msg;
        }

        void slamPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "SLAM Pose Callback");
            iSLAMPoseMsgArrived_b = true;
            iROSSLAMPositionMsg_msg = *pMsg_msgp;
            iROSSLAMCogPositionMsg_msg = iROSSLAMPositionMsg_msg;
        }

        void gnssCovarianceCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "GNSS Covariance Callback");
            iGNSSCovarianceMsgArrived_b = true;
            iROSGNSSCovarianceMsg_msg = *pMsg_msgp;
        }

        void slamCovarianceCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "SLAM Covariance Callback");
            iSLAMCovarianceMsgArrived_b = true;
            iROSSLAMCovarianceMsg_msg = *pMsg_msgp;
        }

        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "IMU Callback");
            struct timeval lTimeval_tv;
            iIMUMsgArrived_b = true;
            iROSIMUMsg_msg = *pMsg_msgp;
            gettimeofday(&lTimeval_tv, NULL);
            iIMUMsgArriveTime_u64 = 
                (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
                (unsigned long long)(lTimeval_tv.tv_usec) / 1000;
        }

        void vehicleCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Vehicle Callback");
            struct timeval lTimeval_tv;
            iVehicleStatusMsgArrived_b = true;
            iROSVehicleStatusMsg_msg = *pMsg_msgp;
            gettimeofday(&lTimeval_tv, NULL);
            iVehicleStatusMsgArriveTime_u64 = 
                (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
                (unsigned long long)(lTimeval_tv.tv_usec) / 1000;
        }

        void autonomousModeCallback(const std_msgs::msg::Bool::ConstSharedPtr pMsg_msgp)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Autonomous Mode Callback");
            iROSAutonomousModeMsg_msg = *pMsg_msgp;
            iPrevDrivingMode_i32 = iDrivingMode_i32;
            if (iROSAutonomousModeMsg_msg.data) {
                iDrivingMode_i32 = 1;
            } else {
                iDrivingMode_i32 = 0;
            }
        }

        void timerCallback()
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Timer Callback");
            // RCLCPP_INFO_STREAM(this->get_logger(), "Pose Msg Arrived: " << iPoseMsgArrived_b);
            visualization_msgs::msg::Marker lROSEstAccuracyMarker_msg;
            geometry_msgs::msg::PoseStamped lROSEstPoseCog_msg;
            geometry_msgs::msg::PoseStamped lROSEstPoseBaselink_msg;
            std_msgs::msg::Float32 lROSEstTravDistOdom_msg;
            std_msgs::msg::Float32 lROSEstTravDistEstPos_msg;
            double lAccuracyScaleFactor_d = 10;

            struct timeval lTimeval_tv;
            gettimeofday(&lTimeval_tv, NULL);
            unsigned long long lCurrentTime_u64 = 
                (unsigned long long)(lTimeval_tv.tv_sec) * 1000 +
                (unsigned long long)(lTimeval_tv.tv_usec) / 1000;

            if (  ((lCurrentTime_u64 - iVehicleStatusMsgArriveTime_u64) < iROSParamMsgTimeout_d) &&
                  ((lCurrentTime_u64 - iIMUMsgArriveTime_u64) < iROSParamMsgTimeout_d) && 
                  ((iGNSSPoseMsgArrived_b && iROSParamGNSSAvailable_b) || (iSLAMPoseMsgArrived_b && iROSParamSLAMAvailable_b) || (!(iROSParamGNSSAvailable_b || iROSParamSLAMAvailable_b)) || iROSParamDoNotWaitForPosMsgs_b) &&
                  iIMUMsgArrived_b && iVehicleStatusMsgArrived_b) {
                sModelStates lCurrentModelStates_st;
                double lTmpRoll_d = 0;
                double lTmpPitch_d = 0;
                double lTmpYaw_d = 0;

                if ( (iROSGNSSCogPositionMsg_msg.pose.orientation.x != 0) || (iROSGNSSCogPositionMsg_msg.pose.orientation.y != 0) || (iROSGNSSCogPositionMsg_msg.pose.orientation.z != 0) || (iROSGNSSCogPositionMsg_msg.pose.orientation.w != 0) ) {
                    tf2::Quaternion lTmpQuternion_c(
                        iROSGNSSCogPositionMsg_msg.pose.orientation.x,
                        iROSGNSSCogPositionMsg_msg.pose.orientation.y,
                        iROSGNSSCogPositionMsg_msg.pose.orientation.z,
                        iROSGNSSCogPositionMsg_msg.pose.orientation.w);
                    tf2::Matrix3x3 lTmpMatrix(lTmpQuternion_c);

                    lTmpMatrix.getRPY(lTmpRoll_d, lTmpPitch_d, lTmpYaw_d);
                } else {
                    if ((iPositionEstimation_cl.getFiltMeasOri() != INVALID_ORIENTATION)) {
                        lTmpYaw_d = iPositionEstimation_cl.getFiltMeasOri();
                    }
                }
                iPositionEstimation_cl.setMeasuredValuesGNSS(iROSGNSSCogPositionMsg_msg.pose.position.x, iROSGNSSCogPositionMsg_msg.pose.position.y, iROSGNSSCogPositionMsg_msg.pose.position.z, lTmpYaw_d);

                if (iROSParamInvertYawRate_b) {
                    iPositionEstimation_cl.setMeasuredValuesIMU(iROSIMUMsg_msg.linear_acceleration.y, iROSIMUMsg_msg.linear_acceleration.x, iROSIMUMsg_msg.linear_acceleration.z, iROSIMUMsg_msg.angular_velocity.x, iROSIMUMsg_msg.angular_velocity.y, -1*iROSIMUMsg_msg.angular_velocity.z);
                } else {
                    iPositionEstimation_cl.setMeasuredValuesIMU(iROSIMUMsg_msg.linear_acceleration.y, iROSIMUMsg_msg.linear_acceleration.x, iROSIMUMsg_msg.linear_acceleration.z, iROSIMUMsg_msg.angular_velocity.x, iROSIMUMsg_msg.angular_velocity.y, iROSIMUMsg_msg.angular_velocity.z);
                }
                iPositionEstimation_cl.setMeasuredValuesVehicleState((iROSVehicleStatusMsg_msg.twist.angular.z), iROSVehicleStatusMsg_msg.twist.linear.x);

                //int32_t lPrevDrivingMode_i32 = iDrivingMode_i32;
                //iDrivingMode_i32 = 0;//iROSVehicleStatusMsg_msg.drivemode;

                bool lResetEstimation_b = false;
                if (iPrevDrivingMode_i32 != iDrivingMode_i32) {
                    lResetEstimation_b = true;
					iPrevDrivingMode_i32 = iDrivingMode_i32;
                }

                bool lGNSSAvailable_b = iROSParamGNSSAvailable_b && iGNSSPoseMsgArrived_b;
                bool lGNSSCovarianceAvailable_b = iGNSSCovarianceMsgArrived_b;
                bool lSLAMAvailable_b = iROSParamSLAMAvailable_b && iSLAMPoseMsgArrived_b;
                bool lSLAMCovarianceAvailable_b = iSLAMCovarianceMsgArrived_b;
                double lGNSSCovariance_da[3] = {iROSParamGNSSDefaultCovariance_d, iROSParamGNSSDefaultCovariance_d, iROSParamGNSSDefaultCovariance_d};
                double lSLAMCovariance_da[3] = {iROSParamSLAMDefaultCovariance_d, iROSParamSLAMDefaultCovariance_d, iROSParamSLAMDefaultCovariance_d};
                
                if (lGNSSCovarianceAvailable_b) {
                    lGNSSCovariance_da[0] = iROSGNSSCovarianceMsg_msg.position_covariance[0];
                    lGNSSCovariance_da[1] = iROSGNSSCovarianceMsg_msg.position_covariance[4];
                    lGNSSCovariance_da[2] = iROSGNSSCovarianceMsg_msg.position_covariance[8];
                }
                if (lSLAMCovarianceAvailable_b) {
                    lSLAMCovariance_da[0] = iROSSLAMCovarianceMsg_msg.position_covariance[0];
                    lSLAMCovariance_da[1] = iROSSLAMCovarianceMsg_msg.position_covariance[4];
                    lSLAMCovariance_da[2] = iROSSLAMCovarianceMsg_msg.position_covariance[8];
                }

                if (lGNSSAvailable_b) {
                    if ((lGNSSCovariance_da[0] > iROSParamGNSSAccuracyLimit_d) || (lGNSSCovariance_da[1] > iROSParamGNSSAccuracyLimit_d)) {
                        lGNSSAvailable_b = false;
                    }
                }
                if (lSLAMAvailable_b) {
                    if ((lSLAMCovariance_da[0] > iROSParamSLAMAccuracyLimit_d) || (lSLAMCovariance_da[1] > iROSParamSLAMAccuracyLimit_d)) {
                        lSLAMAvailable_b = false;
                    }
                }

                iPositionEstimation_cl.iterateEstimation( iROSParamUseRawModel_b,
                                                          lGNSSAvailable_b,
                                                          lSLAMAvailable_b,
                                                          lGNSSCovariance_da,
                                                          lSLAMCovariance_da,
                                                          lResetEstimation_b);
                iPositionEstimation_cl.getModelStates(&lCurrentModelStates_st);

/*
    RCLCPP_INFO_STREAM(this->get_logger(),
						"--Model EKF Beta: " << lCurrentModelStates_st.iBeta_d <<
						"  YR: " << lCurrentModelStates_st.iYawRate_d << 
						"  YA: " << lCurrentModelStates_st.iYawAngle_d << 
						"  LA: " << lCurrentModelStates_st.iLateralAcceleration_d << 
						"  X: " << lCurrentModelStates_st.iPositionX_d << 
						"  Y: " << lCurrentModelStates_st.iPositionY_d << 
						"  VX: " << lCurrentModelStates_st.iLongitudinalVelocity_d << 
						"  VY: " << lCurrentModelStates_st.iLateralVelocity_d);


    RCLCPP_INFO_STREAM(this->get_logger(),
						"--Mes: " << 
                        "  X: "<< iROSGNSSCogPositionMsg_msg.pose.position.x << 
                        "  Y: " << iROSGNSSCogPositionMsg_msg.pose.position.y << 
                        "  Z: " << iROSGNSSCogPositionMsg_msg.pose.position.z <<
                        "  yaw: " << lTmpYaw_d <<
						"  Ax: " << iROSIMUMsg_msg.linear_acceleration.x <<
                        "  Ay: " << iROSIMUMsg_msg.linear_acceleration.y <<
                        "  Az: " << iROSIMUMsg_msg.linear_acceleration.z <<
                        "  AngVx: " << iROSIMUMsg_msg.angular_velocity.x << 
                        "  AngVy: " << iROSIMUMsg_msg.angular_velocity.y <<
                        "  AngVz: " << iROSIMUMsg_msg.angular_velocity.z <<
                        "  SW: " << iROSVehicleStatusMsg_msg.twist.angular.z <<
                        "  Vx: " << iROSVehicleStatusMsg_msg.twist.linear.x*1);
*/

                lAccuracyScaleFactor_d = iPositionEstimation_cl.getAccuracyScaleFactor();

                tf2::Quaternion lTmpOrientation_cl;
                lTmpOrientation_cl.setRPY(0, 0, lCurrentModelStates_st.iYawAngle_d);
                geometry_msgs::msg::Quaternion lTmpOrientationQuatMsg_msg;
                tf2::convert(lTmpOrientation_cl, lTmpOrientationQuatMsg_msg);
                lROSEstPoseCog_msg.header.stamp = this->get_clock()->now();
                lROSEstPoseCog_msg.header.frame_id = "pose_cog";
                lROSEstPoseCog_msg.pose.position.x = lCurrentModelStates_st.iPositionX_d;
                lROSEstPoseCog_msg.pose.position.y = lCurrentModelStates_st.iPositionY_d;
                lROSEstPoseCog_msg.pose.position.z = 0;
                //lROSEstPoseCog_msg.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(0, 0, lCurrentModelStates_st.iYawAngle1_d);
                lROSEstPoseCog_msg.pose.orientation = lTmpOrientationQuatMsg_msg;
            } else {
                tf2::Quaternion lTmpOrientation_cl;
                lTmpOrientation_cl.setRPY(0, 0, 0);
                geometry_msgs::msg::Quaternion lTmpOrientationQuatMsg_msg;
                tf2::convert(lTmpOrientation_cl, lTmpOrientationQuatMsg_msg);
                lROSEstPoseCog_msg.header.stamp = this->get_clock()->now();
                lROSEstPoseCog_msg.header.frame_id = "pose_cog";
                lROSEstPoseCog_msg.pose.position.x = 0;
                lROSEstPoseCog_msg.pose.position.y = 0;
                lROSEstPoseCog_msg.pose.position.z = 0;
                //lROSEstPoseCog_msg.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
                lROSEstPoseCog_msg.pose.orientation = lTmpOrientationQuatMsg_msg;
            }

            iROSPubEstimatedPoseCog_cl->publish(lROSEstPoseCog_msg);
            lROSEstTravDistOdom_msg.data = iPositionEstimation_cl.getTravDistanceOdom();
            lROSEstTravDistEstPos_msg.data = iPositionEstimation_cl.getTravDistanceEstPos();
            iROSPubEstimatedTraveledDistanceOdom_cl->publish(lROSEstTravDistOdom_msg);
            iROSPubEstimatedTraveledDistanceEstPos_cl->publish(lROSEstTravDistEstPos_msg);
           
            //tf2_ros::Buffer iTransformBuffer_cl;
            std::unique_ptr<tf2_ros::Buffer> iTransformBuffer_cl = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf2::Quaternion lTmpOrientation_cl;
            lTmpOrientation_cl.setRPY(0, 0, 0);
            geometry_msgs::msg::Quaternion lTmpOrientationQuatMsg_msg;
            tf2::convert(lTmpOrientation_cl, lTmpOrientationQuatMsg_msg);
            lROSEstPoseBaselink_msg.header.frame_id = "pose_base_link";
            lROSEstPoseBaselink_msg.header.stamp = this->get_clock()->now();
            lROSEstPoseBaselink_msg.pose.position.x = -iPositionEstimation_cl.getCogDistanceFromBaselinkX();
            lROSEstPoseBaselink_msg.pose.position.y =  iPositionEstimation_cl.getCogDistanceFromBaselinkY();
            lROSEstPoseBaselink_msg.pose.position.z =  iPositionEstimation_cl.getCogDistanceFromBaselinkZ();
            //lROSEstPoseBaselink_msg.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            lROSEstPoseBaselink_msg.pose.orientation = lTmpOrientationQuatMsg_msg;

            geometry_msgs::msg::TransformStamped lEstPose_tr;
            lEstPose_tr.header.frame_id = lROSEstPoseCog_msg.header.frame_id;
            lEstPose_tr.child_frame_id = lROSEstPoseBaselink_msg.header.frame_id;
            lEstPose_tr.transform.translation.x = lROSEstPoseCog_msg.pose.position.x;
            lEstPose_tr.transform.translation.y = lROSEstPoseCog_msg.pose.position.y;
            lEstPose_tr.transform.translation.z = lROSEstPoseCog_msg.pose.position.z;
            lEstPose_tr.transform.rotation = lROSEstPoseCog_msg.pose.orientation;
            iTransformBuffer_cl->setTransform(lEstPose_tr, "default_authority", true);
            geometry_msgs::msg::TransformStamped lTsLookup_cl;
            lTsLookup_cl = iTransformBuffer_cl->lookupTransform( lROSEstPoseCog_msg.header.frame_id, lROSEstPoseBaselink_msg.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(lROSEstPoseBaselink_msg, lROSEstPoseBaselink_msg, lTsLookup_cl);
            lROSEstPoseBaselink_msg.header.frame_id = "pose_base_link";
            lROSEstPoseBaselink_msg.header.stamp = this->get_clock()->now();
            iROSPubEstimatedPoseBaselink_cl->publish(lROSEstPoseBaselink_msg);

            // Broadcast tf2 transform from here
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = this->get_clock()->now();
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = "base_link";
            transformStamped.transform.translation.x = lROSEstPoseBaselink_msg.pose.position.x;
            transformStamped.transform.translation.y = lROSEstPoseBaselink_msg.pose.position.y;
            transformStamped.transform.translation.z = lROSEstPoseBaselink_msg.pose.position.z;
            transformStamped.transform.rotation.x = lROSEstPoseBaselink_msg.pose.orientation.x;
            transformStamped.transform.rotation.y = lROSEstPoseBaselink_msg.pose.orientation.y;
            transformStamped.transform.rotation.z = lROSEstPoseBaselink_msg.pose.orientation.z;
            transformStamped.transform.rotation.w = lROSEstPoseBaselink_msg.pose.orientation.w;

            iEstPosBaselinkTransformBroadcaster_cl->sendTransform(transformStamped);
            lROSEstAccuracyMarker_msg.header.frame_id = "base_link";
            lROSEstAccuracyMarker_msg.header.stamp = this->get_clock()->now();
            lROSEstAccuracyMarker_msg.ns = iROSParamEstimatedPoseBaselinkTopic_s;
            lROSEstAccuracyMarker_msg.id = 0;
            lROSEstAccuracyMarker_msg.type = visualization_msgs::msg::Marker::SPHERE;
            lROSEstAccuracyMarker_msg.action = visualization_msgs::msg::Marker::ADD;
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
            if (iGNSSCovarianceMsgArrived_b) {
                lROSEstAccuracyMarker_msg.scale.x = lAccuracyScaleFactor_d * (sqrt(iROSGNSSCovarianceMsg_msg.position_covariance[0] * iROSGNSSCovarianceMsg_msg.position_covariance[4]))/2;
                lROSEstAccuracyMarker_msg.scale.y = lAccuracyScaleFactor_d * (sqrt(iROSGNSSCovarianceMsg_msg.position_covariance[0] * iROSGNSSCovarianceMsg_msg.position_covariance[4]))/2;
                lROSEstAccuracyMarker_msg.scale.z = lAccuracyScaleFactor_d * (sqrt(iROSGNSSCovarianceMsg_msg.position_covariance[0] * iROSGNSSCovarianceMsg_msg.position_covariance[4]))/2;            
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
            iROSPubAccuracyMarker_cl->publish( lROSEstAccuracyMarker_msg );
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";
            for (const auto &param : parameters)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
                if (param.get_name() == "autonomous_mode_topic")
                {
                    iROSParamAutonomousModeTopic_s = param.as_string();;
                    iROSSubAutonomousMode_cl = this->create_subscription<std_msgs::msg::Bool>(iROSParamAutonomousModeTopic_s, 1000, std::bind(&KalmanPosNode::autonomousModeCallback, this, std::placeholders::_1)); 
                }
                if (param.get_name() == "gnss_pose_topic")
                {
                    iROSParamGNSSPoseTopic_s = param.as_string();;
                    iROSSubGNSSPose_cl = this->create_subscription<geometry_msgs::msg::PoseStamped>(iROSParamGNSSPoseTopic_s, 1000, std::bind(&KalmanPosNode::gnssPoseCallback, this, std::placeholders::_1));
                }
                if (param.get_name() == "slam_pose_topic")
                {
                    iROSParamSLAMPoseTopic_s = param.as_string();;
                    iROSSubSLAMPose_cl = this->create_subscription<geometry_msgs::msg::PoseStamped>(iROSParamSLAMPoseTopic_s, 1000, std::bind(&KalmanPosNode::slamPoseCallback, this, std::placeholders::_1));
                }
                if (param.get_name() == "vehicle_status_topic")
                {
                    iROSParamVehicleStatusTopic_s = param.as_string();
                    iROSSubVehicleStatus_cl = this->create_subscription<geometry_msgs::msg::TwistStamped>(iROSParamVehicleStatusTopic_s, 1000, std::bind(&KalmanPosNode::vehicleCallback, this, std::placeholders::_1));
                }
                if (param.get_name() == "nav_sat_fix_topic")
                {
                    iROSParamGNSSCovarianceTopic_s = param.as_string();
                    //iROSSubNavSatFix_cl = this->create_subscription<sensor_msgs::msg::NavSatFix>(iROSParamGNSSCovarianceTopic_s, 1000, std::bind(&KalmanPosNode::navSatFixCallback, this, std::placeholders::_1));
                }
                if (param.get_name() == "imu_topic")
                {
                    iROSParamImuTopic_s = param.as_string();
                    iROSSubIMU_cl = this->create_subscription<sensor_msgs::msg::Imu>(iROSParamImuTopic_s, 1000, std::bind(&KalmanPosNode::imuCallback, this, std::placeholders::_1));
                }
                if (param.get_name() == "est_cog_topic")
                {
                    iROSParamEstimatedPoseCogTopic_s = param.as_string();
                    iROSPubEstimatedPoseCog_cl = this->create_publisher<geometry_msgs::msg::PoseStamped>(iROSParamEstimatedPoseCogTopic_s, 1000);
                }
                if (param.get_name() == "est_baselink_topic")
                {
                    iROSParamEstimatedPoseBaselinkTopic_s = param.as_string();
                    iROSPubEstimatedPoseBaselink_cl = this->create_publisher<geometry_msgs::msg::PoseStamped>(iROSParamEstimatedPoseBaselinkTopic_s, 1000);
                }
                if (param.get_name() == "est_accuracy_topic")
                {
                    iROSParamEstimationAccuracyTopic_s = param.as_string();
                    iROSPubAccuracyMarker_cl = this->create_publisher<visualization_msgs::msg::Marker>(iROSParamEstimationAccuracyTopic_s, 1000);
                }
                if (param.get_name() == "est_trav_distance_odom_topic")
                {
                    iROSParamEstimatedTravDistOdom_s = param.as_string();
                    iROSPubEstimatedTraveledDistanceOdom_cl = this->create_publisher<std_msgs::msg::Float32>(iROSParamEstimatedTravDistOdom_s, 1000);
                }
                if (param.get_name() == "est_trav_distance_est_pos_topic")
                {
                    iROSParamEstimatedTravDistEstPos_s = param.as_string();
                    iROSPubEstimatedTraveledDistanceEstPos_cl = this->create_publisher<std_msgs::msg::Float32>(iROSParamEstimatedTravDistEstPos_s, 1000);
                }
                if (param.get_name() == "loop_rate_hz")
                {
                    iROSParamLoopRateHz_i32 = param.as_int();
                    timer_->cancel(); //
                    double iROSParamLoopRateHz_dbl = static_cast<double>(iROSParamLoopRateHz_i32);
                    int millisec = (1/iROSParamLoopRateHz_dbl) * 1000;
                    RCLCPP_INFO_STREAM(this->get_logger(), "Loop Rate: " << iROSParamLoopRateHz_i32 << " Hz");
                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(millisec),
                        std::bind(&KalmanPosNode::timerCallback, this));
                }
                if (param.get_name() == "gnss_available")
                {
                    iROSParamGNSSAvailable_b = param.as_bool();
                }
                if (param.get_name() == "slam_available")
                {
                    iROSParamSLAMAvailable_b = param.as_bool();
                }
                if (param.get_name() == "gnss_accuracy_limit")
                {
                    iROSParamGNSSAccuracyLimit_d = param.as_double();
                }
                if (param.get_name() == "slam_accuracy_limit")
                {
                    iROSParamSLAMAccuracyLimit_d = param.as_double();
                }
                if (param.get_name() == "gnss_default_covariance")
                {
                    iROSParamGNSSDefaultCovariance_d = param.as_double();
                }
                if (param.get_name() == "slam_default_covariance")
                {
                    iROSParamSLAMDefaultCovariance_d = param.as_double();
                }
                if (param.get_name() == "use_raw_model")
                {
                    iROSParamUseRawModel_b = param.as_bool();
                }
                if (param.get_name() == "orientation_est_enabled")
                {
                    iOriEstimationEnabled_b = param.as_bool();
                }
                if (param.get_name() == "dynamic_time_calc")
                {
                    iROSParamDynamicTimeCalcEnabled_b = param.as_bool();
                }
                if (param.get_name() == "do_not_wait_for_gnss_msgs")
                {
                    iROSParamDoNotWaitForPosMsgs_b = param.as_bool();
                }
                if (param.get_name() == "kinematic_model_max_speed")
                {
                    iROSParamKinematicModelMaxSpeed_d = param.as_double();
                }
                if (param.get_name() == "msg_timeout")
                {
                    iROSParamMsgTimeout_d = param.as_double();
                }
                if (param.get_name() == "invert_yaw_rate")
                {
                    iROSParamInvertYawRate_b = param.as_bool();
                }
                if (param.get_name() == "vehicle_param_c1")
                {
                    iROSParamVehicleParamC1_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_c2")
                {
                    iROSParamVehicleParamC2_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_m")
                {
                    iROSParamVehicleParamM_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_jz")
                {
                    iROSParamVehicleParamJz_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_l1")
                {
                    iROSParamVehicleParamL1_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_l2")
                {
                    iROSParamVehicleParamL2_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_l2")
                {
                    iROSParamVehicleParamL2_d = param.as_double();
                }
                if (param.get_name() == "vehicle_param_swr")
                {
                    iROSParamVehicleParamSWR_d = param.as_double();
                }
            }
            return result;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};


int main(int argc, char **argv)
{
    // Initialize ROS node
    rclcpp::init(argc, argv);
    auto lROSNodeHandle_cl = std::make_shared<KalmanPosNode>();
    RCLCPP_INFO_ONCE(lROSNodeHandle_cl->get_logger(), "ROS2::START");
    rclcpp::spin(lROSNodeHandle_cl);
    rclcpp::shutdown();
    return 0;
}
