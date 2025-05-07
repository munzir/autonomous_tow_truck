import rclpy
from rclpy.node import Node
import cv2
import serial
from std_msgs.msg import Int8

class LightingConditionNode(Node):
    def __init__(self):
        super().__init__('lighting_condition_node')

        # Publishers
        self.lighting_pub = self.create_publisher(Int8, 'lighting_condition', 10)

        # Serial connection to Arduino
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Change to match your port
            self.get_logger().info("Connected to Arduino.")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port to Arduino.")
            self.arduino = None

        # Initialize camera
        self.cap = cv2.VideoCapture(6)
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

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        avg_brightness = v_channel.mean()

        lighting_condition = 1 if avg_brightness < 50 else 0  # 1 = Dark

        self.lighting_pub.publish(Int8(data=lighting_condition))

        # Send to Arduino
        if self.arduino:
            try:
                self.arduino.write(f'{lighting_condition}\n'.encode())
            except Exception as e:
                self.get_logger().error(f"Failed to write to Arduino: {e}")

        self.get_logger().info(f"Lighting Condition (Binary): {lighting_condition}")

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        if self.arduino:
            self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    node = LightingConditionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
