from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="kalman_pos", 
            executable='kalman_pos_node', 
            output='screen',
            parameters=[
                {"gnss_pose_topic" : "/lexus3/gps/duro/current_pose"},
                {"slam_pose_topic" : "/lexus3/gps/duro/current_pose"},
                {"vehicle_status_topic" : "/lexus3/vehicle_status"},
                {"nav_sat_fix_topic" : "gps/duro/fix"},
                {"imu_topic" : "/lexus3/gps/duro/imu"},
                {"est_cog_topic" : "estimated_pose_cog"},
                {"est_baselink_topic" : "estimated_pose_baselink"},
                {"est_accuracy_topic" : "estimation_accuracy"},
                {"est_trav_distance_odom_topic" : "distance"},
                {"est_trav_distance_est_pos_topic" : "estimated_trav_dist_est_pos"},
                {"loop_rate_hz" : 60},
                {"gnss_available", False}
                {"slam_available", False}
                {"gnss_accuracy_limit" : 10.0},
                {"slam_accuracy_limit" : 10.0},
                {"dynamic_time_calc" : True},
                {"do_not_wait_for_gnss_msgs" : True},
                {"kinematic_model_max_speed" : 0.3},
                {"use_raw_model", False}
                {"orientation_est_enabled", False}
                {"msg_timeout" : 2000.0},
                {"vehicle_param_c1" : 3000.0},
                {"vehicle_param_c2" : 3000.0},
                {"vehicle_param_m" : 180.0},
                {"vehicle_param_jz" : 270.0},
                {"vehicle_param_l1" : 0.624},
                {"vehicle_param_l2" : 0.676},
                {"vehicle_param_swr" : 1.0}
            ]
        ),
    ])


