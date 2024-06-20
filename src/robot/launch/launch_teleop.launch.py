from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',  # Replace 'robot' with your actual package name
            executable='teleop_keyboard.py',  # Replace with your Python executable name
            name='teleop_node',
            output='screen',
        ),
    ])
