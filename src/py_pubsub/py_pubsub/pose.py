import rclpy  # ROS 2 client library for Python
from std_msgs.msg import Float32  # For publishing frequency and steering angle in this format
import serial  # To use Serial (in Arduino IDE) for communication

def main():
    rclpy.init()  # Boot rclpy for use
    node = rclpy.create_node('ros_arduino_communication')  # Create the actual node
    pub_float_frequency = node.create_publisher(Float32, 'frequency_data', 10)  # Publisher for frequency data
    pub_float_steering_angle = node.create_publisher(Float32, 'steering_angle_data', 10)  # Publisher for steering angle data

    # Establish a serial connection to the first Arduino (frequency sensor)
    try:
        arduino_frequency = serial.Serial('/dev/ttyACM1', 9600)  # Update the port as needed
        node.get_logger().info('Connected to frequency sensor Arduino on /dev/ttyACM1')
    except serial.SerialException as e:
        node.get_logger().error(f'Failed to connect to frequency sensor Arduino: {e}')
        return

    # Establish a serial connection to the second Arduino (steering angle sensor)
    try:
        arduino_steering = serial.Serial('/dev/ttyACM0', 9600)  # Update the port as needed
        node.get_logger().info('Connected to steering angle sensor Arduino on /dev/ttyACM0')
    except serial.SerialException as e:
        node.get_logger().error(f'Failed to connect to steering angle sensor Arduino: {e}')
        return

    try:
        while rclpy.ok():
            # Read frequency data
            if arduino_frequency.in_waiting > 0:
                line = arduino_frequency.readline().decode('utf-8').rstrip()
                try:
                    frq = float(line)  # Convert the frequency data to float
                    msg_float = Float32()
                    msg_float.data = frq
                    pub_float_frequency.publish(msg_float)  # Publish frequency data
                    node.get_logger().info(f'Published frequency data: {frq}')
                except ValueError:
                    node.get_logger().warn(f'Received invalid frequency data: {line}')

            # Read steering angle data
            if arduino_steering.in_waiting > 0:
                line = arduino_steering.readline().decode('utf-8').rstrip()
                try:
                    steering_angle = float(line)  # Convert the steering angle data to float
                    msg_float = Float32()
                    msg_float.data = steering_angle
                    pub_float_steering_angle.publish(msg_float)  # Publish steering angle data
                    node.get_logger().info(f'Published steering angle data: {steering_angle}')
                except ValueError:
                    node.get_logger().warn(f'Received invalid steering angle data: {line}')

    except KeyboardInterrupt:
        pass  # Allow exiting with Ctrl-C

    finally:
        # Clean up connections
        if arduino_frequency.is_open:
            arduino_frequency.close()
        if arduino_steering.is_open:
            arduino_steering.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  # Standard to run the file

