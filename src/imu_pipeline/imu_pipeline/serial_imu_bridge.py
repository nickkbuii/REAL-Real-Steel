"""
ROS2 node that:
- Connects to a serial IMU device.
- Reads and parses incoming IMU data packets.
- Publishes the parsed IMU data to two separate ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time

class SerialIMUNode(Node):
    def __init__(self):
        super().__init__("serial_imu_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)

        PORT = self.get_parameter("port").get_parameter_value().string_value
        BAUD = self.get_parameter("baud").get_parameter_value().integer_value

        self.pub_upper = self.create_publisher(Imu, "/upper_imu/data_raw", 10)
        self.pub_fore  = self.create_publisher(Imu, "/forearm_imu/data_raw", 10)

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info(f"Connected to {PORT} @ {BAUD}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            raise SystemExit

        self.timer = self.create_timer(0.005, self.read_serial)

    def parse_packet(self, line):
        """
        Parse a CSV IMU packet of 12 floats.
        """
        if "," not in line:
            return None

        parts = line.split(",")
        if len(parts) != 12:
            return None

        try:
            return list(map(float, parts))
        except:
            return None

    def read_serial(self):
        try:
            raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
        except:
            return

        if not raw:
            return

        vals = self.parse_packet(raw)

        if vals is None:
            self.get_logger().warn(f"Bad packet: {raw}")
            return

        try:
            (
                ax1, ay1, az1, gx1, gy1, gz1,
                ax2, ay2, az2, gx2, gy2, gz2
            ) = vals
        except:
            self.get_logger().warn(f"Parse error: {raw}")
            return

        t = self.get_clock().now().to_msg()

        # Upper IMU
        imu1 = Imu()
        imu1.header.stamp = t
        imu1.header.frame_id = "upper_imu"
        imu1.linear_acceleration.x = ax1
        imu1.linear_acceleration.y = ay1
        imu1.linear_acceleration.z = az1
        imu1.angular_velocity.x = gx1
        imu1.angular_velocity.y = gy1
        imu1.angular_velocity.z = gz1
        self.pub_upper.publish(imu1)

        # Forearm IMU
        imu2 = Imu()
        imu2.header.stamp = t
        imu2.header.frame_id = "forearm_imu"
        imu2.linear_acceleration.x = ax2
        imu2.linear_acceleration.y = ay2
        imu2.linear_acceleration.z = az2
        imu2.angular_velocity.x = gx2
        imu2.angular_velocity.y = gy2
        imu2.angular_velocity.z = gz2
        self.pub_fore.publish(imu2)


def main(args=None):
    rclpy.init(args=args)
    node = SerialIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
