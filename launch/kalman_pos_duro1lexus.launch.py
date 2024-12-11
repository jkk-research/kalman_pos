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
                {"gnss_covariance_topic" : "gps/duro/fix"},
                {"slam_covariance_topic" : "gps/duro/fix"},
                {"imu_topic" : "/lexus3/gps/duro/imu"},
                {"est_cog_topic" : "estimated_pose_cog"},
                {"est_baselink_topic" : "estimated_pose_baselink"},
                {"est_accuracy_topic" : "estimation_accuracy"},
                {"est_trav_distance_odom_topic" : "distance"},
                {"est_trav_distance_est_pos_topic" : "estimated_trav_dist_est_pos"},
                {"loop_rate_hz" : 60},
                {"gnss_available", False},
                {"slam_available", False},
                {"gnss_accuracy_limit" : 10.0},
                {"slam_accuracy_limit" : 10.0},
                {"gnss_default_covariance" : 15.0},
                {"slam_default_covariance" : 15.0},
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
        Node(
            package='kalman_pos',
            executable='vehicle_status_convert',
            output='screen',
            parameters=[
                {"speed_topic": "/nissan/vehicle_speed"},
                {"steer_topic": "/nissan/vehicle_steering"},
                {"status_topic": "/nissan/vehicle_status"},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_imu_ned_enu',
            output='screen',
            arguments=['1.5385', '0.0', '-0.325', '0.0', '0', '0.0', 'imu_link_ned', 'duro'] # https://github.com/szenergy/szenergy-public-resources/wiki/H-sensorset2020.A
        ),
        Node(
            package='imu_transformer',
            executable='imu_transformer_node',
            name='imu_data_transformer',
            output='screen',
            remappings=[
                ('imu_in', '/nissan/gps/duro/imu'),
                ('imu_out', '/nissan/gps/duro/imu_cog'),
                ('mag_in', '/nissan/gps/duro/mag'),
                ('mag_out', '/nissan/gps/duro/mag_cog')
            ],
            parameters=[
                {'target_frame': 'duro'}
            ]
        ),
    ])


