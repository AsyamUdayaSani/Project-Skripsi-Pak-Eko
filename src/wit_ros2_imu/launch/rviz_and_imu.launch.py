from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_and_imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        remappings=[('/wit/imu', '/imu/data_raw')],
        parameters=[{'port': '/dev/ttyUSB1'},
                    {"baud": 9600}],
        output="screen"
    )
    return LaunchDescription([rviz_and_imu_node])
