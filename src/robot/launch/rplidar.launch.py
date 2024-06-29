import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            namespace='',
            output='screen',
            parameters=[
                {
                    'angle_compensate': True,
                    'channel_type': 'serial',
                    'flip_x_axis': False,
                    'frame_id': 'laser_frame',
                    'inverted': False,
                    'scan_mode': 'Standard',  # Update with your desired scan mode if applicable
                    'serial_baudrate': 115200,
                    'serial_port': '/dev/ttyUSB0',
                    'topic_name': 'scan',
                    'use_sim_time': False,
                    'auto_standby': True
                }
            ]
        )
    ])

