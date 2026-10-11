import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_type     = LaunchConfiguration('channel_type',     default='serial')
    serial_port = LaunchConfiguration(
        'serial_port',
        default='/dev/ttyUSB1'
    )
    serial_baudrate  = LaunchConfiguration('serial_baudrate',  default='460800')
    frame_id         = LaunchConfiguration('frame_id',         default='laser')
    inverted         = LaunchConfiguration('inverted',         default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode',        default='Standard')

    return LaunchDescription([
        DeclareLaunchArgument('channel_type',     default_value=channel_type),
        DeclareLaunchArgument('serial_port',      default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate',  default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id',         default_value=frame_id),
        DeclareLaunchArgument('inverted',         default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode',        default_value=scan_mode),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type':     channel_type,
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         frame_id,
                'inverted':         inverted,
                'angle_compensate': angle_compensate,
		'scan_frequency': 12.0
            }],
            output='screen'
        ),
    ])
