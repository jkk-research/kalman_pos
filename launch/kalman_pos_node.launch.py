from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "kalman_pos", package='kalman_pos', executable='kalman_pos_node', output='screen',
            parameters=[
                {"pose_topic":"gps/duro/current_pose"},
                {"vehicle_status_topic":"vehicle_status"},
                {"nav_sat_fix_topic":"gps/duro/status_string"},
                {"inspvax_topic":"gps/nova/inspvax"},
                {"imu_topic":"imu/data"},
                {"est_cog_topic":"estimated_pose_cog"},
                {"est_trav_distance_odom_topic":"distance"},
                {"est_trav_distance_est_pos_topic":"estimated_trav_dist_est_pos"},
                {"est_baselink_topic":"estimated_pose_baselink"},
                {"est_accuracy_topic":"estimation_accuracy"},
                {"loop_rate_hz":60},
                {"estimation_method":6},
                {"gnss_source":"none"},
                {"vehicle_type":"SZEmission"},
                {"dynamic_time_calc":True},
                {"kinematic_model_max_speed":0.3},
                {"do_not_wait_for_gnss_msgs":True},
                {"msg_timeout":2000},
                {"vehicle_param_c1":3000},
                {"vehicle_param_c2":3000},
                {"vehicle_param_m":180},
                {"vehicle_param_jz":270},
                {"vehicle_param_l1":0.624},
                {"vehicle_param_l2":0.676},
                {"vehicle_param_swr":1}
            ]
        ),
    ])


