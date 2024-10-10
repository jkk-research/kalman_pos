from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="kalman_pos", 
            executable='kalman_pos_node', 
            output='screen',
            parameters=[
                {"pose_topic" : "/lexus3/gps/duro/current_pose"},
                {"vehicle_status_topic" : "/lexus3/vehicle_status"},
                # {"nav_sat_fix_topic" : "/lexus3/gps/duro/status_string"},
                # {"inspvax_topic" : "gps/nova/inspvax"},
                {"imu_topic" : "/lexus3/gps/duro/imu"},
                {"est_cog_topic" : "estimated_pose_cog"},
                {"est_trav_distance_odom_topic" : "distance"},
                {"est_trav_distance_est_pos_topic" : "estimated_trav_dist_est_pos"},
                {"est_baselink_topic" : "estimated_pose_baselink"},
                {"est_accuracy_topic" : "estimation_accuracy"},
                {"loop_rate_hz" : 60},
                {"estimation_method" : 10},
                {"gnss_source" : "none"},
                {"vehicle_type" : "lexus"},
                {"dynamic_time_calc" : True},
                {"kinematic_model_max_speed" : 0.3},
                {"do_not_wait_for_gnss_msgs" : True},
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


