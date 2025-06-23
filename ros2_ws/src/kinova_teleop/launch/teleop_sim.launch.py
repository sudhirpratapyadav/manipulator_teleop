from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinova_teleop',
            executable='joystick_interface.py',
            name='joystick_interface',
            output='screen',
            prefix='python3'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='kinova_teleop',
            executable='test_ee_sim.py',
            name='test_ee_sim',
            output='screen',
            prefix='python3'
        )
    ])
