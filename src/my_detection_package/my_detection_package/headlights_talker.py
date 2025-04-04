import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Int8, Float32

class LightingConditionNode(Node):
    def __init__(self):
        super().__init__('lighting_condition_node')
        
        # Publishers
        self.lighting_pub = self.create_publisher(Int8, 'lighting_condition', 10)  # Binary lighting condition
        # self.brightness_pub = self.create_publisher(Float32, 'average_brightness', 10)  # Raw brightness value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(6)  # Change index if needed
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            return

        # Timer to process frames
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame!")
            return

        # Convert frame to HSV and extract brightness (V channel)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        avg_brightness = v_channel.mean()

        # Determine binary lighting condition
        lighting_condition = 1 if avg_brightness < 50 else 0  # 1 = Dark, 0 = Normal/Bright

        # Publish messages
        self.lighting_pub.publish(Int8(data=lighting_condition))
        # self.brightness_pub.publish(Float32(data=avg_brightness))

        # Log output
        self.get_logger().info(f"Lighting Condition (Binary): {lighting_condition}")

        # Show the frame
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = LightingConditionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
