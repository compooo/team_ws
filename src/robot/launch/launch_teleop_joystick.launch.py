import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    joystick = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('robot'), 'launch', 'joystick.launch.py')]))
    return LaunchDescription([
        Node(
            package='robot',  # Replace 'robot' with your actual package name
            executable='teleop_keyboard.py',  # Replace with your Python executable name
            name='teleop_node',
            output='screen',
        ),
        joystick
    ])
