"""Launch mapping 3D dengan koreksi IMU, untuk rig yang dipasang di drone.

Contoh:
    ros2 launch sweep_mappingimu sweep_mappingimu.launch.py sweeps:=1
    ros2 launch sweep_mappingimu sweep_mappingimu.launch.py sweeps:=1 rpm:=2.0

Menyalakan: IMU -> LiDAR -> stepper_sweep_node -> mapping_3d_imu -> foxglove_bridge.

Stepper dan bag recorder dipanggil dari paket sweep_mapping, bukan disalin ke
sini. Jadi perubahan pada keduanya (misalnya perhitungan RPM) otomatis berlaku
untuk kedua launch file dan tidak bisa menyimpang diam-diam.
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
            description='Detik per step motor. Makin kecil makin cepat. '
                        'Cara lama, tetap berlaku.',
        ),
        DeclareLaunchArgument(
            'rpm',
            default_value='0.0',
            description='Kecepatan putar piringan LiDAR (putaran per menit), '
                        'alternatif yang lebih mudah dibaca. Dipakai hanya '
                        'kalau diisi > 0; kalau tidak, `delay` yang menentukan.',
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
            description='true = rotasi -sudut (sama seperti sweep_mapping).',
        ),
        DeclareLaunchArgument(
            'publish_every_sweep',
            default_value='true',
            description='Publish cloud kumulatif tiap sweep selesai, bukan cuma di akhir.',
        ),
        DeclareLaunchArgument(
            'tilt_offset_deg',
            default_value='0.0',
            description='Koreksi kemiringan tetap (derajat) terhadap sumbu putar '
                        'stepper, kalau posisi awal LiDAR tidak benar-benar datar.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='Topic sensor_msgs/Imu yang dipakai untuk koreksi.',
        ),
        DeclareLaunchArgument(
            'use_yaw',
            default_value='false',
            description='Ikut mengoreksi yaw. Default mati karena yaw berasal '
                        'dari magnetometer, yang kacau di dekat motor drone. '
                        'Nyalakan hanya kalau uji lapangan menunjukkan bersih.',
        ),
        DeclareLaunchArgument(
            'imu_roll_offset_deg',
            default_value='0.0',
            description='Bias pemasangan IMU pada sumbu roll. Isi dari angka '
                        'yang dilaporkan node saat rig berdiri di meja datar.',
        ),
        DeclareLaunchArgument(
            'imu_pitch_offset_deg',
            default_value='0.0',
            description='Bias pemasangan IMU pada sumbu pitch. Isi dari angka '
                        'yang dilaporkan node saat rig berdiri di meja datar.',
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
            default_value='scanimu',
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
        # Versi 360 derajat. mapping_3d_imu belum membuang fase pulang milik
        # stepper_sweep_node (versi 180 derajat), jadi jangan ditukar dulu.
        executable='stepper_sweep_node1',
        name='stepper_sweep_node',
        output='screen',
        parameters=[{
            'num_sweeps': ParameterValue(
                LaunchConfiguration('sweeps'), value_type=int),
            'delay': ParameterValue(
                LaunchConfiguration('delay'), value_type=float),
            'rpm': ParameterValue(
                LaunchConfiguration('rpm'), value_type=float),
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
        package='sweep_mappingimu',
        executable='mapping_3d_imu',
        name='mapping_3d_imu',
        output='screen',
        parameters=[{
            'invert_rotation': ParameterValue(
                LaunchConfiguration('invert_rotation'), value_type=bool),
            'publish_every_sweep': ParameterValue(
                LaunchConfiguration('publish_every_sweep'), value_type=bool),
            'tilt_offset_deg': ParameterValue(
                LaunchConfiguration('tilt_offset_deg'), value_type=float),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_yaw': ParameterValue(
                LaunchConfiguration('use_yaw'), value_type=bool),
            'imu_roll_offset_deg': ParameterValue(
                LaunchConfiguration('imu_roll_offset_deg'), value_type=float),
            'imu_pitch_offset_deg': ParameterValue(
                LaunchConfiguration('imu_pitch_offset_deg'), value_type=float),
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
