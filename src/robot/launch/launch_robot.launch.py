import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Specify the path to your URDF Xacro file
    robot_xacro_file = os.path.join(
        get_package_share_directory('robot'),
        'urdf',
        'robot.xacro'
    )

    # Convert Xacro to URDF and load it into the parameter server
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_xacro_file,
        description='Location of robot description file')

    xacro_file = ExecuteProcess(
        cmd=['xacro', '--inorder', robot_xacro_file], 
        output='screen'
    )

    # Launch Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(
            get_package_share_directory('robot'),
            'worlds', 'obstacles.world'
        )}.items(),
    )

    # Spawn the robot model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', LaunchConfiguration('robot_description'),
                   '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    return LaunchDescription([
        robot_description,
        xacro_file,
        gazebo,
        spawn_entity
    ])
