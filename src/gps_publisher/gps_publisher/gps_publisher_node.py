import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import re
import time

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'fix', 10)
        self.filename = "/home/lyeba/Desktop/COM1___38400_250225_075057.txt"

        # Timer to check file updates
        self.timer = self.create_timer(0.5, self.read_nmea_file)

    def convert_nmea_to_decimal(self, degrees_minutes, direction):
        match = re.match(r'(\d+)(\d\d\.\d+)', degrees_minutes)
        if not match:
            return None

        degrees = float(match.group(1))
        minutes = float(match.group(2))
        decimal_degrees = degrees + (minutes / 60)

        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees

        return decimal_degrees

    def read_nmea_file(self):
        try:
            with open(self.filename, "r", encoding="utf-8", errors="ignore") as file:
                file.seek(0, 2)  # Move to the end of the file

                while True:
                    line = file.readline()
                    if not line:
                        time.sleep(0.5)
                        continue

                    line = line.strip()

                    if line.startswith("$GNGGA"):
                        parts = line.split(",")

                        if len(parts) > 9 and parts[2] and parts[4]:  # Ensure valid lat/lon
                            latitude = self.convert_nmea_to_decimal(parts[2], parts[3])
                            longitude = self.convert_nmea_to_decimal(parts[4], parts[5])

                            gps_msg = NavSatFix()
                            gps_msg.header.stamp = self.get_clock().now().to_msg()
                            gps_msg.header.frame_id = "gps"
                            gps_msg.latitude = latitude
                            gps_msg.longitude = longitude
                            gps_msg.altitude = 0.0  # Altitude not required

                            self.publisher_.publish(gps_msg)
                            self.get_logger().info(f"📍 Published: Lat={latitude:.6f}, Lon={longitude:.6f}")

        except FileNotFoundError:
            self.get_logger().error(f"❌ Error: File '{self.filename}' not found!")
        except Exception as e:
            self.get_logger().error(f"⚠️ Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
