"""Launch mapping 3D dengan jumlah sweep yang bisa diatur.

Contoh:
    ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=3

Menyalakan: IMU -> LiDAR -> stepper_sweep_node -> mapping_3d_sweep -> foxglove_bridge.
Paket wit_ros2_imu, sllidar_ros2, dan stepper_controller tidak diubah sama sekali;
launch ini hanya memanggil launch file mereka yang sudah ada.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'sweeps',
            default_value='1',
            description='Jumlah sweep (1 sweep = LiDAR muter 360 derajat). '
                        '0 = muter terus tanpa henti.',
        ),
        DeclareLaunchArgument(
            'delay',
            default_value='0.018',
            description='Detik per step motor. Makin kecil makin cepat.',
        ),
        DeclareLaunchArgument(
            'direction',
            default_value='1',
            description='Arah putar motor: 1 atau 0.',
        ),
        DeclareLaunchArgument(
            'steps_per_rev',
            default_value='1600',
            description='Step motor per satu putaran motor (microstepping driver).',
        ),
        DeclareLaunchArgument(
            'gear_ratio',
            default_value='0.5',
            description='Pulley motor / pulley LiDAR. 30T/60T = 0.5.',
        ),
        DeclareLaunchArgument(
            'return_home_on_abort',
            default_value='true',
            description='Kalau Ctrl+C di tengah sweep, motor muter balik ke posisi awal.',
        ),
        DeclareLaunchArgument(
            'invert_rotation',
            default_value='true',
            description='true = rotasi -sudut (sama seperti mapping_3d.py yang lama).',
        ),
        DeclareLaunchArgument(
            'publish_every_sweep',
            default_value='true',
            description='Publish cloud kumulatif tiap sweep selesai, bukan cuma di akhir.',
        ),
        DeclareLaunchArgument(
            'foxglove',
            default_value='true',
            description='Nyalakan foxglove_bridge untuk lihat hasil dari laptop.',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Otomatis ros2 bag record, berhenti sendiri saat sweep selesai.',
        ),
        DeclareLaunchArgument(
            'bag_dir',
            default_value='~/bags',
            description='Folder tempat menyimpan hasil rekaman.',
        ),
        DeclareLaunchArgument(
            'bag_prefix',
            default_value='scan',
            description='Awalan nama bag. Hasil: <prefix>_0001_<n>sweep.',
        ),
        DeclareLaunchArgument(
            'compress',
            default_value='false',
            description='Kompres rekaman dengan zstd (file lebih kecil, CPU lebih berat).',
        ),
    ]

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('wit_ros2_imu'),
                'launch',
                'rviz_and_imu.launch.py',
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py',
            )
        )
    )

    stepper_node = Node(
        package='sweep_mapping',
        executable='stepper_sweep_node',
        name='stepper_sweep_node',
        output='screen',
        parameters=[{
            'num_sweeps': ParameterValue(
                LaunchConfiguration('sweeps'), value_type=int),
            'delay': ParameterValue(
                LaunchConfiguration('delay'), value_type=float),
            'direction': ParameterValue(
                LaunchConfiguration('direction'), value_type=int),
            'steps_per_rev': ParameterValue(
                LaunchConfiguration('steps_per_rev'), value_type=int),
            'gear_ratio': ParameterValue(
                LaunchConfiguration('gear_ratio'), value_type=float),
            'return_home_on_abort': ParameterValue(
                LaunchConfiguration('return_home_on_abort'), value_type=bool),
        }],
    )

    mapping_node = Node(
        package='sweep_mapping',
        executable='mapping_3d_sweep',
        name='mapping_3d_sweep',
        output='screen',
        parameters=[{
            'invert_rotation': ParameterValue(
                LaunchConfiguration('invert_rotation'), value_type=bool),
            'publish_every_sweep': ParameterValue(
                LaunchConfiguration('publish_every_sweep'), value_type=bool),
        }],
    )

    recorder_node = Node(
        package='sweep_mapping',
        executable='bag_recorder_node',
        name='bag_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
        parameters=[{
            'bag_dir': LaunchConfiguration('bag_dir'),
            'bag_prefix': LaunchConfiguration('bag_prefix'),
            'num_sweeps': ParameterValue(
                LaunchConfiguration('sweeps'), value_type=int),
            'compress': ParameterValue(
                LaunchConfiguration('compress'), value_type=bool),
        }],
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription(args + [
        imu_launch,
        lidar_launch,
        stepper_node,
        mapping_node,
        recorder_node,
        foxglove_node,
    ])
