from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_behaviours_manager',
            executable='system_state_mocker',
            name='system_state_mocker',
            remappings=[
                ('~/system_state', '/system_state')
            ],
        ),
        Node(
            package='system_behaviours_manager',
            executable='ros_behaviour_tree',
            emulate_tty=True,
            output='screen',
        )
    ])