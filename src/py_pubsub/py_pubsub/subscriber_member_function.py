import rclpy  # Again for ROS 2 usage
from rclpy.node import Node  # For inheritance
from std_msgs.msg import Float32  # For publishing frequency and steering angle in this format
import math  # For trigonometry in general

class KinematicCalculator(Node):  # This class is inheriting from Node, which is from ROS 2

    def __init__(self):
        super().__init__('kinematic_calculator')  # Name of our node to run the constructor as ROS 2 requires

        # Subscriber for frequency data
        self.frequency_subscription = self.create_subscription(
            Float32,  # The data type of the published data
            'frequency_data',  # The name
            self.frequency_callback,  # Callback function
            10)  # Buffer size

        # Subscriber for steering angle data
        self.steering_angle_subscription = self.create_subscription(
            Float32,  # The data type of the published data
            'steering_angle_data',  # The name
            self.steering_angle_callback,  # Callback function
            10)  # Buffer size

        # Publishers for the calculated values
        self.pub_x = self.create_publisher(Float32, 'x', 10)
        self.pub_y = self.create_publisher(Float32, 'y', 10)
        self.pub_theta = self.create_publisher(Float32, 'theta', 10)
        self.pub_v = self.create_publisher(Float32, 'v', 10)
        # Initialize variables for cumulative values
        #self.x0 = 0.0
        #self.y0 = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Initial heading angle (radians)
        self.v=0.0
        # Initialize variables for received data
        self.frequency = 0.0
        self.phi = 0.0  # Default steering angle (will be updated by the callback)
        
        self.phi_array = []
        
    def steering_angle_callback(self, msg):
        # Append the received phi value to the array
        self.phi = msg.data
        self.phi_array.append(msg.data)
        
    def frequency_callback(self, msg):
        self.frequency = msg.data
        self.calculate_and_publish()

    #def steering_angle_callback(self, msg):
        #self.phi = msg.data  # Update steering angle with the received data
	#self.phi_array.append(msg.data)
	
    def calculate_and_publish(self):
    	
    	# Calculate average phi if phi_array is not empty
        if self.phi_array:
            self.phi = ( (sum(self.phi_array) / len(self.phi_array)))  # Average the phi values
            self.phi_array = []  # Clear the array after averaging

        # Constants
        wheel_radius = 0.2032  # Radius of the wheel (meters)
        wheelbase = 1.17  # Distance between front and back wheels (meters)
        pulses_per_revolution = 740*1.1
        sampling_time = 0.2  # Sampling time (seconds)

        # Calculate the wheel speed (v)
        self.v = (self.frequency * 2 * math.pi * wheel_radius) / pulses_per_revolution

        # Update the heading angle (theta)
        self.theta += (self.v / wheelbase) * math.tan(math.radians(self.phi)) * sampling_time

        # Update x, y positions using integration over time
        self.x += self.v * math.cos(self.theta) * sampling_time 
        self.y += self.v * math.sin(self.theta) * sampling_time
	
	# Convert the x,y positions to the world frame
	#self.x= self.x0 * math.cos(self.theta) - self.y0 * math.sin(self.theta)
	#self.y= self.x0 * math.sin(self.theta) + self.y0 * math.cos(self.theta)
	
        # Publish the cumulative values 
        x_msg = Float32()
        x_msg.data = self.x
        self.pub_x.publish(x_msg)
        self.get_logger().info(f'Published x: {self.x}')

        y_msg = Float32()
        y_msg.data = self.y
        self.pub_y.publish(y_msg)
        self.get_logger().info(f'Published y: {self.y}')

        theta_msg = Float32()
        theta_msg.data = self.theta
        self.pub_theta.publish(theta_msg)
        self.get_logger().info(f'Published theta: {self.theta}')
        
        v_msg = Float32()
        v_msg.data = self.v
        self.pub_v.publish(v_msg)
        self.get_logger().info(f'Published v: {self.v}')

def main(args=None): 
    rclpy.init(args=args)  # Initialize
    kinematic_calculator = KinematicCalculator()  # Create our object
    rclpy.spin(kinematic_calculator)  # Spinning our node i.e. executing work
    kinematic_calculator.destroy_node()  # Destroy the node and shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()  # Standard to run the file

