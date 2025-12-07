#!/usr/bin/env python3
"""
ROS2 node that:
- Connects to the NanoESP32 over BLE
- Receives IMU CSV data (12 floats, SI units)
- Prints IMU values exactly like bluetooth.py
- Publishes ROS2 Imu messages to:
      upper_imu/data_raw
      forearm_imu/data_raw
"""

import asyncio
from bleak import BleakScanner, BleakClient
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuBleNode(Node):
    """Converts BLE IMU packets into ROS2 Imu messages."""

    def __init__(self):
        super().__init__("imu_ble_node")
        self.pub_upper = self.create_publisher(Imu, "upper_imu/data_raw", 10)
        self.pub_fore  = self.create_publisher(Imu, "forearm_imu/data_raw", 10)

    def make_msg(self, ax, ay, az, gx, gy, gz, frame_id):
        """Create a ROS2 Imu message given SI acceleration and angular velocity."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance[0] = -1.0
        return msg

    def process(self, data: bytes):
        """Parse 12-float CSV IMU payload, print, and publish ROS2 messages."""
        text = data.decode("utf-8").strip()
        parts = text.split(",")

        if len(parts) != 12:
            print("Unexpected payload length:", len(parts), "data:", text)
            return

        (ax1, ay1, az1,
         gx1, gy1, gz1,
         ax2, ay2, az2,
         gx2, gy2, gz2) = map(float, parts)

        print(f"Upper IMU  a[m/s^2]=({ax1:.3f}, {ay1:.3f}, {az1:.3f})"
              f"  w[rad/s]=({gx1:.3f}, {gy1:.3f}, {gz1:.3f})")
        print(f"Forearm IMU a[m/s^2]=({ax2:.3f}, {ay2:.3f}, {az2:.3f})"
              f"  w[rad/s]=({gx2:.3f}, {gy2:.3f}, {gz2:.3f})")

        self.pub_upper.publish(self.make_msg(ax1, ay1, az1, gx1, gy1, gz1, "upper_imu"))
        self.pub_fore.publish(self.make_msg(ax2, ay2, az2, gx2, gy2, gz2, "forearm_imu"))


async def ble_loop(node: ImuBleNode):
    """Scan for NanoESP32, connect, and forward BLE IMU notifications into ROS2."""
    print("Scanning for NanoESP32...")

    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name and "NanoESP32" in d.name
    )

    if not device:
        print("NanoESP32 not found. Make sure it's powered and advertising.")
        return

    print(f"Found device: {device.name} @ {device.address}")

    async with BleakClient(device.address) as client:
        print("Connected!")

        await client.start_notify(
            "00002a57-0000-1000-8000-00805f9b34fb",
            lambda s, d: node.process(d)
        )

        print("Listening for BLE notifications... (Ctrl+C to stop)")

        while rclpy.ok():
            await asyncio.sleep(0.05)


def main():
    """Initialize ROS2 and run BLE event loop."""
    rclpy.init()
    node = ImuBleNode()

    try:
        asyncio.run(ble_loop(node))
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()