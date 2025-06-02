#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import serial
# import time

# class ModePublisher(Node):
#     def __init__(self, port='/dev/ttyACM0', baud_rate=9600):
#         super().__init__('arduino_mode_publisher')
#         self.get_logger().info("ModePublisher node started.")

#         self.serial_port = serial.Serial(port, baud_rate, timeout=1)
#         self.publisher_ = self.create_publisher(String, '/mode_switch', 10)
#         self.timer = self.create_timer(0.1, self.read_serial)
#         self.get_logger().info("Hi, 2")

#     def read_serial(self):
#         if self.serial_port.in_waiting > 0:
#             self.get_logger().info("Hi, 3")
#             line = self.serial_port.readline().decode('utf-8').strip()
#             print(f"Serial data: {line}")
#             self.get_logger().info(f"Read from serial: {line}")


#             if line in ["A", "T", "M"]:
#                 msg = String()
#                 msg.data = line
#                 self.publisher_.publish(msg)
#                 self.get_logger().info(f"Published mode: {line}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ModePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import subprocess

class ModePublisher(Node):
    def __init__(self):
        super().__init__('arduino_mode_publisher')

        # Setup serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino. Starting ROS 2 node and serial handler...")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        # ROS Publisher
        self.publisher_ = self.create_publisher(String, '/mode_switch', 10)
        self.timer = self.create_timer(0.1, self.read_serial)

        # State variables
        self.screen = 0
        self.current_option = ''
        self.mode = 'A'
        self.project_launched = False

        # Destination to script mapping
        self.destination_scripts = {
            'Assemble Area': 'assembly_shop_waypoints.sh',
            'U-Turn': 'u_turn_waypoints.sh',
            'Charging Station': 'charging_station_waypoints.sh',
            'Bumper Shop': 'bumper_shop_waypoints.sh',
        }

    def run_script(self, cmd):
        try:
            with open('logs.txt', 'a') as log_file:
                subprocess.Popen(cmd, stdout=log_file, stderr=log_file, shell=True)
        except Exception as e:
            self.get_logger().error(f"Failed to run script '{cmd}': {e}")

    def launch_project_stack(self):
        if not self.project_launched:
            self.get_logger().info("Launching main project stack...")
            self.run_script("launch_project.sh")
            self.project_launched = True

    def read_serial(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if not line:
                    return

                self.get_logger().info(f"Received from Arduino: {line}")

                if line.startswith("Screen:"):
                    parts = line.split(",")
                    self.screen = int(parts[0].split(":")[1].strip())
                    self.current_option = parts[1].split(":")[1].strip()

                elif line.startswith("Option:"):
                    self.current_option = line.split(":", 1)[1].strip()

                elif line == "3":  # Confirm button
                    if self.screen == 0:
                        script = self.destination_scripts.get(self.current_option)
                        if script:
                            self.launch_project_stack()
                            self.get_logger().info(f"Launching script for: {self.current_option}")
                            self.run_script(script)
                        else:
                            self.get_logger().warn(f"No script mapped for: {self.current_option}")
                    elif self.screen == 1:
                        self.get_logger().info(f"Confirmed number of dollies: {self.current_option}")

                elif line in ["A", "T", "M"]:
                    self.mode = line
                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published mode: {line}")
                    self.run_script(f"./mode_switch.sh {line}")

                elif line == "C":
                    self.get_logger().info("Confirmation timeout. Cleared.")

        except Exception as e:
            self.get_logger().error(f"Error in serial read: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisher()
    try:
        rclpy.spin(node)  # This keeps the node alive!
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
