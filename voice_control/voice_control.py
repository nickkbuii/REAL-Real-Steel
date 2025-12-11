#!/usr/bin/env python3
"""
ROS2 node: maps recognized voice commands (/voice_cmd) to predefined UR7e
trajectories and publishes them as JointTrajectory messages with joint limits.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

UR7E_LIMITS = {
    "j1": (-3.14, 3.14),
    "j2": (-3.14, 3.14),
    "j3": (-3.14, 3.14),
    "j4": (-3.14, 3.14),
    "j5": (-3.14, 3.14),
    "j6": (-3.14, 3.14),
}


def clamp(x, lo, hi):
    return max(lo, min(x, hi))

# NOTE: Arbitary trajectories, replace with actual trajectories
TRAJ = {
    "jab": [
        ([0.0, -1.5, 1.5, -1.8, 1.6, 0.0], 0.0),
        ([0.1, -1.4, 1.3, -1.7, 1.6, 0.0], 0.25),
        ([0.0, -1.5, 1.5, -1.8, 1.6, 0.0], 0.50),
    ],
    "uppercut": [
        ([0.2, -1.4, 1.4, -1.6, 1.5, 0.0], 0.0),
        ([0.3, -1.1, 1.1, -1.4, 1.5, 0.0], 0.35),
        ([0.2, -1.4, 1.4, -1.6, 1.5, 0.0], 0.60),
    ],
    "hook": [
        ([0.0, -1.4, 1.3, -1.6, 1.6, 0.0], 0.0),
        ([0.4, -1.4, 1.3, -1.6, 1.6, 0.0], 0.20),
        ([0.0, -1.4, 1.3, -1.6, 1.6, 0.0], 0.45),
    ],
    "reset": [
        ([0.0, -1.57, 1.57, -1.57, 1.57, 0.0], 0.0)
    ],
}


class voice_control(Node):
    """Publishes UR7e trajectories when voice commands match keywords."""

    def __init__(self):
        super().__init__("voice_control")
        self.sub = self.create_subscription(String, "/voice_cmd", self.cb, 10)
        self.pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )

    def cb(self, msg):
        text = msg.data.lower()
        for key in TRAJ:
            if key in text:
                self.play(key)
                break

    def play(self, name):
        traj = JointTrajectory()
        traj.joint_names = JOINTS

        for pos, t in TRAJ[name]:
            clamped = [
                clamp(pos[0], *UR7E_LIMITS["j1"]),
                clamp(pos[1], *UR7E_LIMITS["j2"]),
                clamp(pos[2], *UR7E_LIMITS["j3"]),
                clamp(pos[3], *UR7E_LIMITS["j4"]),
                clamp(pos[4], *UR7E_LIMITS["j5"]),
                clamp(pos[5], *UR7E_LIMITS["j6"]),
            ]

            pt = JointTrajectoryPoint()
            pt.positions = clamped
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            traj.points.append(pt)

        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = voice_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()