#!/usr/bin/env python3

"""
ROS2 node that:
- Subscribes to upper and forearm IMU messages
- Computes arm joint angles
- Publishes JointTrajectory messages to control a UR7e robot arm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R
import numpy as np

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

class IMUToUR7e(Node):
    """
    IMU → UR7e teleoperation node.

    IMU orientation axes (both IMUs):
      - x-axis: along right arm/forearm pointing toward the hand
      - y-axis: pointing left
      - z-axis: pointing up

    Zero pose (for calibration):
      - Right arm straight forward (shoulder flexed forward, elbow extended)
      - Hold this pose for the first ~3 seconds after startup
    """

    def __init__(self):
        super().__init__('imu_to_ur7e')

        self.upper_sub = self.create_subscription(
            Imu, '/upper_imu/data', self.upper_cb, 10)
        self.forearm_sub = self.create_subscription(
            Imu, '/forearm_imu/data', self.forearm_cb, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)

        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.upper_q = None
        self.forearm_q = None

        self.upper_q0 = None
        self.forearm_q0 = None
        self.calibrated = False
        self.calib_samples = []

        self.initial_j1 = 4.726624011993408
        self.initial_j2 = -1.8424164829649867
        self.initial_j3 = -1.4260427951812744
        self.initial_j4 = -1.3948713105968018
        self.initial_j5 = 1.5788230895996094
        self.initial_j6 = -3.1389945189105433


        self.joint_states_ready = False
        self.timer = self.create_timer(0.1, self.publish_robot_motion)

        self.get_logger().info(
            "IMU → UR7e teleop started.\n"
            "HOLD RIGHT ARM STRAIGHT FORWARD FOR CALIBRATION"
        )

    def joint_state_cb(self, msg: JointState):
        self.joint_states_ready = True

    def upper_cb(self, msg: Imu):
        self.upper_q = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])

    def forearm_cb(self, msg: Imu):
        self.forearm_q = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])


    def compute_arm_angles(self):
        if self.upper_q is None or self.forearm_q is None:
            return None

        if not self.calibrated:
            self.calib_samples.append((self.upper_q.copy(), self.forearm_q.copy()))

            if len(self.calib_samples) >= 10:
                uppers = np.array([u for (u, _) in self.calib_samples])
                forearms = np.array([f for (_, f) in self.calib_samples])

                self.upper_q0 = np.mean(uppers, axis=0)
                self.upper_q0 /= np.linalg.norm(self.upper_q0)

                self.forearm_q0 = np.mean(forearms, axis=0)
                self.forearm_q0 /= np.linalg.norm(self.forearm_q0)

                self.calibrated = True
                self.get_logger().info("IMU calibration complete.")

            return None

        upper_r_raw = R.from_quat(self.upper_q)
        forearm_r_raw = R.from_quat(self.forearm_q)

        upper_r0 = R.from_quat(self.upper_q0)
        forearm_r0 = R.from_quat(self.forearm_q0)

        upper_r = upper_r0.inv() * upper_r_raw
        forearm_r = forearm_r0.inv() * forearm_r_raw

        roll_u, pitch_u, yaw_u = upper_r.as_euler('xyz')
        shoulder_yaw = yaw_u
        shoulder_pitch = pitch_u

        rel_r = upper_r.inv() * forearm_r
        _, pitch_rel, _ = rel_r.as_euler('xyz')
        elbow_flex = pitch_rel

        return shoulder_yaw, shoulder_pitch, elbow_flex

    def publish_robot_motion(self):
        if not self.joint_states_ready:
            return

        angles = self.compute_arm_angles()
        if angles is None:
            return

        shoulder_yaw, shoulder_pitch, elbow_flex = angles
        self.get_logger().info(
            f"cmd angles: yaw={shoulder_yaw:.3f}, pitch={shoulder_pitch:.3f}, elbow={elbow_flex:.3f}"
        )

        j1 = self.initial_j1 + clamp(shoulder_yaw, *UR7E_LIMITS["j1"])
        j2 = self.initial_j2 + clamp(-shoulder_pitch, *UR7E_LIMITS["j2"])
        j3 = self.initial_j3 + clamp(-elbow_flex, *UR7E_LIMITS["j3"])
        j4 = self.initial_j4
        j5 = self.initial_j5
        j6 = self.initial_j6

        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        pt = JointTrajectoryPoint()
        pt.positions = [j1, j2, j3, j4, j5, j6]

        pt.velocities = [0.0]*6
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 10 * (10**6)

        traj.points.append(pt)
        self.pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToUR7e()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()