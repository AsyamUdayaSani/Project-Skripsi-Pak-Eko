from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stepper_controller',
            executable='stepper_node',
            name='stepper_node',
            parameters=[
                {'steps_per_rev': 1600},
                {'delay': 0.001},
                {'direction': 1}
            ],
            output='screen'
        )
    ])
