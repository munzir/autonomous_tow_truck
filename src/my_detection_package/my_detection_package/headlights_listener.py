import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class LightingConditionListener(Node):
    def __init__(self):
        super().__init__('lighting_condition_listener')

        # Subscribe to the lighting condition topic
        self.lighting_sub = self.create_subscription(
            Int8, 'lighting_condition', self.lighting_callback, 10)

    def lighting_callback(self, msg):
        print(msg.data)  # Print only the binary value (0 or 1)

def main(args=None):
    rclpy.init(args=args)
    node = LightingConditionListener()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
