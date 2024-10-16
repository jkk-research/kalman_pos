from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace= "kalman_pos", package='kalman_pos', executable='kalman_pos_node', output='screen',
            parameters=[
                {"pose_topic":"/nissan/gps/duro/current_pose"},
                {"vehicle_status_topic":"/nissan/vehicle_status"},
                {"nav_sat_fix_topic":"/nissan/gps/duro/status_string"},
                # {"inspvax_topic":"gps/nova/inspvax"},
                {"imu_topic":"/nissan/gps/duro/imu_cog"},
                {"est_cog_topic":"estimated_pose_cog"},
                {"est_trav_distance_odom_topic":"distance"},
                {"est_trav_distance_est_pos_topic":"estimated_trav_dist_est_pos"},
                {"est_baselink_topic":"estimated_pose_baselink"},
                {"est_accuracy_topic":"estimation_accuracy"},
                {"loop_rate_hz":60},
                {"estimation_method":6},
                {"gnss_source":"none"},
                {"vehicle_type":"nissan"},
                {"dynamic_time_calc":True},
                {"kinematic_model_max_speed":3.0},
                {"do_not_wait_for_gnss_msgs":True},
                {"msg_timeout":2000.0},
                {"vehicle_param_c1":30000.0},
                {"vehicle_param_c2":30000.0},
                {"vehicle_param_m":1920.0},
                {"vehicle_param_jz":27000.0},
                {"vehicle_param_l1":1.1615},
                {"vehicle_param_l2":1.5385},
                {"vehicle_param_swr":1.0}
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


