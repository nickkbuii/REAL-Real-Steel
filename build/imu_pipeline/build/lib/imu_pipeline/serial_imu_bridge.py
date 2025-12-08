import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time


class SerialIMUNode(Node):
    def __init__(self):
        super().__init__("serial_imu_bridge")

        # ---------- ROS PARAMETERS ----------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)

        PORT = self.get_parameter("port").get_parameter_value().string_value
        BAUD = self.get_parameter("baud").get_parameter_value().integer_value

        # ---------- PUBLISHERS ----------
        self.pub_upper = self.create_publisher(Imu, "upper_imu/data_raw", 10)
        self.pub_fore  = self.create_publisher(Imu, "forearm_imu/data_raw", 10)

        # ---------- SERIAL ----------
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info(f"Connected to {PORT} @ {BAUD}")
        except Exception as e:
            self.get_logger().fatal(f"FAILED to open serial port: {e}")
            raise SystemExit

        # ---------- TIMER (200 Hz) ----------
        self.timer = self.create_timer(0.005, self.read_serial)

    # ----------------------------------------------------
    def parse_packet(self, line):
        """
        Handles BOTH of these formats:

        CSV:
        -0.21,0.02,9.80,0.00,0.00,0.00,-0.01,-0.05,9.82,0.00,0.00,0.00

        LABELED:
        ax1:-0.21 ay1:0.02 az1:9.80 gx1:-0.00 ...
        """

        # --- Case 1: CSV ---
        if "," in line:
            parts = line.split(",")
            if len(parts) != 12:
                return None
            try:
                return list(map(float, parts))
            except:
                return None

        # --- Case 2: Labeled key:value ---
        clean = line.replace(":", " ").split()
        vals = []

        for token in clean:
            try:
                vals.append(float(token))
            except:
                pass

        if len(vals) != 12:
            return None

        return vals

    # ----------------------------------------------------
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

        # ---------- Upper IMU ----------
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

        # ---------- Forearm IMU ----------
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


# --------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SerialIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
