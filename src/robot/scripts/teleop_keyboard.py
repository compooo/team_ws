#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist  # Message for desired robot velocity

class MotorControlNode(Node):
    wheel_base = 0.3

    def __init__(self):
        super().__init__('motor_control_node')
        self.get_logger().info("Motor control node started")

        # ROS parameters (adjust values as needed)
        self.linear_vel = self.declare_parameter('linear_vel', 0.1).value  # m/s
        self.angular_vel = self.declare_parameter('angular_vel', 1.0).value  # rad/s

        # Serial communication (replace with your specific settings)
        self.serial_port = '/dev/ttyACM0'  # Replace with your serial port name
        self.baud_rate = 115200  # Adjust baud rate if needed
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        # Subscribe to desired velocity topic
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10)  # Adjust queue size if needed

    def velocity_callback(self, msg):
        # Calculate individual motor speeds based on kinematics (differential drive)
        left_motor_speed = msg.linear.x + msg.angular.z * self.wheel_base / 2
        right_motor_speed = msg.linear.x - msg.angular.z * self.wheel_base / 2

        # Print information to terminal
        self.get_logger().info(f"Received velocity: linear={msg.linear.x}, angular={msg.angular.z}")
        self.get_logger().info(f"Calculated speeds: left={left_motor_speed}, right={right_motor_speed}")

        # Convert speeds to control signals suitable for Arduino (replace with your logic)
        left_motor_control = int(left_motor_speed * 255)  # Example: Scale to 0-255
        right_motor_control = int(right_motor_speed * 255)

        self.get_logger().info(f"Calculated speeds: left={left_motor_control}, right={right_motor_control}")


        # Send control signals to Arduino via serial communication
        command_string = f"P,{left_motor_control},{right_motor_control}\n"
        self.ser.write(command_string.encode())
        self.get_logger().debug(f"Sending command: {command_string}")
        self.get_logger().info(f"Received: {' '.join(command_string)}")


def main():
    rclpy.init()
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()