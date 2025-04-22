#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ModePublisher(Node):
    def __init__(self, port='/dev/ttyACM0', baud_rate=9600):
        super().__init__('arduino_mode_publisher')

        self.serial_port = serial.Serial(port, baud_rate, timeout=1)
        self.publisher_ = self.create_publisher(String, '/mode_switch', 10)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line in ["A", "T", "M"]:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published mode: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()