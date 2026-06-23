from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('wit_ros2_imu'),
                'rviz_and_imu.launch.py'
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        )
    )

    stepper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stepper_controller'),
                'launch',
                'stepper_launch.py'
            )
        )
    )

    mapping_node = Node(
        package='stepper_controller',
        executable='mapping_3d',
        name='mapping_3d',
        output='screen'
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    return LaunchDescription([
        imu_launch,
        lidar_launch,
        stepper_launch,
        mapping_node,
        foxglove_node
    ])