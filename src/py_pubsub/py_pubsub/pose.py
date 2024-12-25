import rclpy  # ROS 2 client library for Python
from std_msgs.msg import Float32  # For publishing frequency and steering angle in this format
import serial  # To use Serial (in Arduino IDE) for communication

def restart_serial(node, serial_dev_name, baud_rate):
    device_connected = False
    while (device_connected == False):
        # Establish a serial connection to the first Arduino (frequency sensor)
        try:
            serial_handler = serial.Serial(serial_dev_name, baud_rate)  # Update the port as needed
            node.get_logger().info('Connected to frequency sensor Arduino on /dev/arduino_wheels')
            device_connected = True
        except serial.SerialException as e:
            node.get_logger().error(f'Failed to connect to frequency sensor Arduino: {e}')
            
    return serial_handler

def main():
    rclpy.init()  # Boot rclpy for use
    node = rclpy.create_node('ros_arduino_communication')  # Create the actual node
    pub_float_frequency = node.create_publisher(Float32, 'frequency_data', 10)  # Publisher for frequency data
    pub_float_steering_angle = node.create_publisher(Float32, 'steering_angle_data', 10)  # Publisher for steering angle data

    arduino_frequency = restart_serial(node, '/dev/arduino_wheels', 115200) 
    arduino_steering = restart_serial(node, '/dev/arduino_steering', 115200)

    try:
        while rclpy.ok():
            # Read frequency data
            try:
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
            except Exception as e:
                node.get_logger().error(f'Unexpected error: {str(e)}')
                arduino_frequency = restart_serial(node, '/dev/arduino_wheels', 115200) 

                # Read steering angle data
            try:
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
            except Exception as e:
                node.get_logger().error(f'Unexpected error: {str(e)}')
                arduino_steering = restart_serial(node, '/dev/arduino_steering', 115200)

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

