import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200


class SerialIMUNode(Node):
    def __init__(self):
        super().__init__("serial_imu_bridge")

        self.pub_upper = self.create_publisher(Imu, "upper_imu/data_raw", 10)
        self.pub_fore  = self.create_publisher(Imu, "forearm_imu/data_raw", 10)

        self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)

        self.get_logger().info(f"Connected to {PORT} @ {BAUD}")

        self.timer = self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        line = self.ser.readline().decode("utf-8").strip()
        if not line:
            return

        parts = line.split(",")
        if len(parts) != 12:
            self.get_logger().warn(f"Bad packet: {line}")
            return

        try:
            vals = list(map(float, parts))
        except:
            self.get_logger().warn(f"Parse fail: {line}")
            return

        ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2 = vals

        t = self.get_clock().now().to_msg()

        # Upper IMU msg
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

        # Forearm IMU msg
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