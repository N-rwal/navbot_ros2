from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_dir = FindPackageShare('navbot_bringup')

    slam_params = PathJoinSubstitution(
        [config_dir, 'config', 'slam.yaml']
    )

    rf2o_params = PathJoinSubstitution(
        [config_dir, 'config', 'rf2o.yaml']
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ldlidar'),
            '/launch/ldlidar.launch.py'
        ])
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '0',     # x
            '0',     # y
            '0.15',  # z
            '0',     # roll
            '0',     # pitch
            '0',     # yaw
            'base_link',
            'laser'
        ]
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params]
    )

    motor_driver = Node(
        package='stm32_omni_driver',
        executable='driver_node',
        name='stm32_omni_driver',
        output='screen',
        parameters=[
            '/home/orangepi/ros2_ws/src/stm32_omni_driver/config/params.yaml'
        ]
    )

    slam = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(
		os.path.join(
		    get_package_share_directory('slam_toolbox'),
		    'launch',
		    'online_async_launch.py'
		)
	    ),
	    launch_arguments={
		'slam_params_file': os.path.join(
		    get_package_share_directory('navbot_bringup'),
		    'config',
		    'slam.yaml'
		),
		'use_sim_time': 'false',
	    }.items()
	)

    return LaunchDescription([
        lidar,
        laser_tf,
        #rf2o,
        motor_driver,
        #slam
    ])
