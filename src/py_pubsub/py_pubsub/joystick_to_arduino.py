import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # Import Twist for cmd_vel topic
import serial
import math
import time
from enum import Enum

# button mapping on xbox joystick
class Button(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    FRONT_LEFT_TOP = 4
    FRONT_RIGHT_TOP = 5
    CENTER_LEFT = 6
    CENTER_RIGHT = 7
    LEFT_THUMB = 9
    RIGHT_THUMB = 10
    CENTER_MIDDLE = 11

# axes mapping in xbox joystick
class Axis(Enum):
    LEFT_THUMB_HORIZONTAL = 0
    LEFT_THUMB_VERTICAL = 1
    FRONT_LEFT_BOTTOM = 2
    RIGHT_THUMB_HORIZONTAL = 3
    RIGHT_THUMB_VERTICAL = 4
    FRONT_RIGHT_BOTTOM = 5
    ARROW_LEFT_RIGHT = 6
    ARROW_UP_DOWN = 7

class JoystickToArduino(Node):
    def __init__(self):
        super().__init__('joystick_to_arduino')

        # ROS subscription to joystick topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            1
        )

        # ROS subscription to cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Serial port initialization for two Arduinos
        self.serial_port_1 = serial.Serial('/dev/ttyACM0', 115200, timeout=5)  # Arduino 1
        #self.serial_port_2 = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Arduino 2
        #self.get_logger().info('Joystick to Arduino node initialized.')

        self.timer = self.create_timer(0.2, self.periodic_log_callback)  # Trigger 

        # start_time = time.time()
        # self.serial_port_2.write(message.encode('utf-8'))
        # end_time = time.time()
        # self.get_logger().info(f"Serial write took {end_time - start_time:.2f} seconds")
        
        # State variables
        self.manual_mode = True
        self.teleop_mode = False
        self.autonom_mode = False
        self.brake_active = False
        self.reverse_mode = False
        self.auto_steering_angle = 0
        self.auto_speed = 0.0
        self.debug_mode = False
        self.reinitialize = False
        self.steering_angle = 0
        # Previous state of the toggle buttons to detect positive edges
        self.prev_manual_button = 0
        self.prev_reverse_button = 0
        self.prev_debug_mode_button = 0

    def joystick_callback(self, msg):
        try:
            # Button mappings
            reinit_button = msg.buttons[Button.A.value]  # Reinitialize all variables
            manual_button = msg.buttons[Button.CENTER_RIGHT.value]  # Hold for manual
            teleop_button = msg.buttons[Button.CENTER_LEFT.value]
            autonom_button = msg.buttons[Button.CENTER_MIDDLE.value]
            brake_button = msg.buttons[Button.B.value]   # Brake button
            reverse_button = msg.buttons[Button.FRONT_LEFT_TOP.value] # Toggle forward/reverse mode
            killswitch_button = msg.buttons[Button.X.value]
            debug_mode_button = msg.buttons[Button.Y.value] 

            # Handle reinitialization
            self.reinitialize = reinit_button == 1

            # Handle manual/teleop toggle (positive edge detection)
            if manual_button == 1:
                self.manual_mode = True
                self.teleop_mode = False
                self.autonom_mode = False
                self.get_logger().info("Manual On!")
            elif teleop_button == 1:
                self.manual_mode = False
                self.teleop_mode = True
                self.autonom_mode = False
                self.get_logger().info("Teleoperation On!")
            elif autonom_button == 1:
                self.manual_mode = False
                self.teleop_mode = False
                self.autonom_mode = True
                self.get_logger().info("Autonomous On!")
            
            # Handle forward/reverse toggle (positive edge detection)
            if reverse_button == 1 and self.prev_reverse_button == 0:
                self.reverse_mode = not self.reverse_mode
                self.get_logger().info(f"Direction = {'Reverse' if self.reverse_mode else 'Forward'}")
            
            
            # Handle debug_mode toggle (positive edge detection)
            if debug_mode_button == 1 and self.prev_debug_mode_button == 0:
                self.debug_mode = not self.debug_mode
                self.get_logger().info(f"Debug Mode = {'True' if self.debug_mode else 'False'}")
            
            # Handle brake
            self.brake_active = brake_button == 1

            # Map joystick index [4] to Speed between 4.4 and 2.5
            raw_speed = msg.axes[Axis.FRONT_RIGHT_BOTTOM.value]
            if raw_speed == 1:
                self.auto_speed = 0
            else:
                self.auto_speed = 4.4 - (raw_speed + 1) * ((4.4 - 2.5) / 2)

            # If brake is active, force speed to 0
            if self.brake_active:
                self.auto_speed = 0

            # Map joystick index [0] to Steering Angle between -255 and 255
            raw_steering = msg.axes[Axis.LEFT_THUMB_HORIZONTAL.value]
            if (self.autonom_mode == True):
                self.steering_angle = self.auto_steering_angle
            else:
                self.steering_angle = int(53 - (raw_steering + 1) * ((53 + 53) / 2))

            # Update previous state for next cycle
            self.prev_manual_button = manual_button
            self.prev_reverse_button = reverse_button
            self.prev_debug_mode_button = debug_mode_button

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")
    
    def periodic_log_callback(self):
        # Periodic logging of state variables
        try: 
             # Format message with all data
            message = (
                f"Reinitialize = {self.reinitialize}, "
                f"Mode = {'Manual' if self.manual_mode else 'Teleoperation' if self.teleop_mode else 'Autonomous' if self.autonom_mode else 'Nothing'}, "
                f"Brake = {self.brake_active}, "
                f"Direction = {'Reverse' if self.reverse_mode else 'Forward'}, "
                f"Speed = {self.auto_speed:.2f}, "
                f"Steering Angle = {self.steering_angle}\n"
                f"Debug Mode = {'True' if self.debug_mode else 'False'}, "
            )
            #time.sleep(1)

            # Send the formatted message to both Arduinos
            start_time = time.time()
            self.serial_port_1.reset_output_buffer()
            self.serial_port_1.write(message.encode('utf-8'))
            end_time = time.time()
            self.get_logger().info(f"Serial write took {end_time - start_time:.2f} seconds")
            self.get_logger().info(f"Sent to Arduino 1: {message.strip()}")

            #self.serial_port_2.write(message.encode('utf-8'))
            # self.get_logger().info(f"Sent to Arduino 2: {message.strip()}")

   
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timeout occurred.")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")
    
    
    
    def cmd_vel_callback(self, msg):
        # Log linear and angular velocities
        self.get_logger().info(
            f"Received /cmd_vel: Linear = {msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f} | "
            f"Angular = {msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f}"
        )
        
        # Extract the required values
        target_linear = msg.linear.x  # Linear speed in the x direction
        target_rot = msg.angular.z   # Angular speed about the z-axis

        # Define wheel_base_ (distance between front and rear axles, adjust as needed)
        wheel_base_ = 1.14  # Replace with your actual wheelbase in meters

        # Calculate the steering angle in degrees
        if target_linear != 0:  # Avoid division by zero
            self.auto_steering_angle = int(180/math.pi * math.atan2(target_rot * wheel_base_, target_linear))
        else:
            self.auto_steering_angle = 0  # If linear speed is 0, assume no steering is need 
            #establish a relation between speed and voltage here TO DO
        # Log the computed steering angle
        self.get_logger().info(f"Computed Steering Angle (degrees): {self.auto_steering_angle:.4f}")

        # Send the linear speed and steering angle to the Arduinos
        message = f"Linear Speed = {target_linear:.2f}, Steering Angle = {self.auto_steering_angle:.4f}\n"
        #self.serial_port_1.write(message.encode('utf-8'))
        #self.serial_port_2.write(message.encode('utf-8'))

        self.get_logger().info(f"Sent to Arduino 1: {message.strip()}")
        self.get_logger().info(f"Sent to Arduino 2: {message.strip()}")

    def destroy_node(self):
        """Ensure proper cleanup."""
        #if self.serial_port_1.is_open:
        #    self.serial_port_1.close()
        #if self.serial_port_2.is_open:
        #    self.serial_port_2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    joystick_to_arduino = JoystickToArduino()

    try:
        rclpy.spin(joystick_to_arduino)
    except KeyboardInterrupt:
        joystick_to_arduino.get_logger().info('Shutting down node.')
    finally:
        joystick_to_arduino.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

