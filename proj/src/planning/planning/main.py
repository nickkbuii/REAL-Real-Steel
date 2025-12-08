#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R
import numpy as np
from python_qt_binding.QtWidgets import QPushButton, QVBoxLayout, QWidget
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from moveit_msgs.msg import RobotTrajectory
from planning.srv import ConfirmPunch

# Import your IKPlanner
from planning.ik import IKPlanner

class UR7ePunch(Node):
    def __init__(self):
        super().__init__('ur7e_punch_node')

        qos = QoSProfile(depth=10)
        self.fist_sub = self.create_subscription(PoseStamped, '/draggable_fist/pose', self.fist_cb, qos)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, qos)
        self.traj_vis_pub = self.create_publisher(RobotTrajectory, '/punch_trajectory_vis', 10)

        self.pub_start = self.create_publisher(PoseStamped, "start_position", 10)
        self.pub_end   = self.create_publisher(PoseStamped, "end_position", 10)

        self.timer = self.create_timer(0.1, self.publish_positions)

        self.start_xyz = [0.324, 0.208, 0.209]
        self.end_xyz   = [0.276, 0.427, 0.656]

        self.exec_ac = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.exec_srv = self.create_service(Trigger, '/execute_punch', self.execute_punch_cb)
        self.exec_srv_confirm = self.create_service(ConfirmPunch, '/confirm_punch', self.confirm_punch_cb)

        self.joint_state = None
        self.fist_pose = None
        self.ik_planner = IKPlanner()

        self._pending_punch = None  # holds approach/punch trajectories
        self.get_logger().info("UR7ePunch node ready")

    # ---------------- callbacks ----------------
    def joint_state_cb(self, msg: JointState):
        self.joint_state = msg

    def fist_cb(self, msg: PoseStamped):
        self.fist_pose = msg

    # def make_pose(self, xyz):
    #     msg = PoseStamped()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = "base_link"
    #     msg.pose.position.x = xyz[0]
    #     msg.pose.position.y = xyz[1]
    #     msg.pose.position.z = xyz[2]

    #     # identity orientation
    #     msg.pose.orientation.w = 1.0
    #     return msg

    # def publish_positions(self):
    #     self.pub_start.publish(self.make_pose(self.start_xyz))
    #     self.pub_end.publish(self.make_pose(self.end_xyz))
        
    def make_point(self, xyz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = xyz[0]
        msg.point.y = xyz[1]
        msg.point.z = xyz[2]
        return msg

    def publish_positions(self):
        self.pub_start.publish(self.make_point(self.start_xyz))
        self.pub_end.publish(self.make_point(self.end_xyz))

    # ---------------- service ----------------
    def execute_punch_cb(self, request, response):
        if self.fist_pose is None:
            response.success = False
            response.message = "No fist pose received yet"
            return response
        if self.joint_state is None:
            response.success = False
            response.message = "No joint state yet"
            return response

        pose = self.fist_pose.pose
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rot_matrix = R.from_quat(quat).as_matrix()
        punch_dir = rot_matrix @ np.array([1, 0, 0])
        punch_dir = punch_dir / np.linalg.norm(punch_dir)

        # approach behind target
        approach_dist = 0.2
        start_pos = position - punch_dir * approach_dist
        self.start_xyz = start_pos
        print(start_pos)
        target_pos = position.copy()
        self.end_xyz = target_pos
        print(target_pos)

        # IK for start and target
        start_js = self.ik_planner.compute_ik(self.joint_state, *start_pos)
        target_js = self.ik_planner.compute_ik(self.joint_state, *target_pos)
        if start_js is None or target_js is None:
            response.success = False
            response.message = "IK failed"
            return response

        # Plan trajectories
        approach_traj = self.ik_planner.plan_to_joints(start_js)
        punch_traj = self.ik_planner.plan_to_joints(target_js)
        if approach_traj is None or punch_traj is None:
            response.success = False
            response.message = "Planning failed"
            return response

        # Safe speed-up for punch
        self._safe_scale_trajectory(punch_traj.joint_trajectory, 2.0)

        # Store trajectories as before
        self._pending_punch = {
            'approach_traj': approach_traj.joint_trajectory,
            'punch_traj': target_js,
            'velocity': 3,
            'stage': 'preview'
        }

        # Publish preview to RViz
        preview_msg = RobotTrajectory()
        preview_msg.joint_trajectory = JointTrajectory()
        # combine approach + punch for preview
        preview_msg.joint_trajectory.points = approach_traj.joint_trajectory.points + punch_traj.joint_trajectory.points
        preview_msg.joint_trajectory.joint_names = approach_traj.joint_trajectory.joint_names
        self.traj_vis_pub.publish(preview_msg)

        self.get_logger().info("Trajectory preview published. Click 'Confirm Punch' to execute.")
        response.success = True
        response.message = "Preview generated"
        return response
        
    def confirm_punch_cb(self, request, response):
        if not self._pending_punch:
            response.success = False
            response.message = "No pending punch trajectory to execute"
            return response
        
        self._pending_punch['velocity'] = getattr(request, 'speed', 3.0)
        if self._pending_punch['velocity'] <= 0:
            self._pending_punch['velocity'] = 3.0

        # Execute approach first
        self._pending_punch['stage'] = 'approach'
        self._execute_joint_trajectory(self._pending_punch['approach_traj'])
        response.success = True
        response.message = "Punch execution started"
        return response

    # ---------------- trajectory helpers ----------------
    def _safe_scale_trajectory(self, traj: JointTrajectory, speed_multiplier: float):
        """
        Proper UR-compatible time-scaling:
        - Shrinks timestamps
        - Recomputes velocities & accelerations
        - Enforces realistic UR joint velocity limits
        """

        # UR5e / UR7e joint velocity limits (rad/s)
        # These are SAFE and correct for all e-series robots:
        vel_limits = [2.16, 2.16, 2.16, 3.15, 3.15, 3.15]

        # ----------------------------
        # 1) Scale timestamps
        # ----------------------------
        for pt in traj.points:
            old_t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            new_t = max(0.01, old_t / speed_multiplier)   # prevent 1ms issues

            pt.time_from_start.sec = int(new_t)
            pt.time_from_start.nanosec = int((new_t % 1.0) * 1e9)

        # ----------------------------
        # 2) Recompute velocities and accelerations
        # ----------------------------
        for i in range(1, len(traj.points)):
            pt = traj.points[i]
            prev = traj.points[i - 1]

            t  = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            tp = prev.time_from_start.sec + prev.time_from_start.nanosec * 1e-9
            dt = max(0.01, t - tp)

            velocities = []
            accelerations = []

            for j in range(len(pt.positions)):
                dq = pt.positions[j] - prev.positions[j]
                v = dq / dt
                velocities.append(v)

            # Second derivative: simple finite difference
            if i >= 2:
                prev_v = traj.points[i-1].velocities
                accels = [(velocities[j] - prev_v[j]) / dt for j in range(len(velocities))]
            else:
                accels = [0.0] * len(velocities)

            pt.velocities = velocities
            pt.accelerations = accels

        # First point has no velocity
        traj.points[0].velocities = [0.0] * len(traj.joint_names)
        traj.points[0].accelerations = [0.0] * len(traj.joint_names)

        # ----------------------------
        # 3) Enforce real UR speed limits
        # ----------------------------
        for pt in traj.points:
            for j in range(len(pt.velocities)):
                pt.velocities[j] = np.clip(pt.velocities[j], -vel_limits[j], vel_limits[j])


    def _execute_joint_trajectory(self, joint_traj: JointTrajectory):
        if not self.exec_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Controller action server not available")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        future = self.exec_ac.send_goal_async(goal)
        future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal send failed: {e}")
            return
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            future.result().result
            self.get_logger().info("Trajectory execution done")
        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            self._pending_punch = None
            return

        if not self._pending_punch:
            return

        stage = self._pending_punch.get('stage')
        if stage == 'approach':
            self.get_logger().info("Approach finished, sending punch...")

            target_js = self._pending_punch['punch_traj']
            punch_traj = self.ik_planner.plan_to_joints(target_js)
            if punch_traj is None:
                response.success = False
                response.message = "Planning failed"
                return response

            # Safe speed-up for punch
            self._safe_scale_trajectory(punch_traj.joint_trajectory, self._pending_punch['velocity'])
            self._pending_punch['stage'] = 'punch'
            # self._execute_joint_trajectory(self._pending_punch['punch_traj'])
            self._execute_joint_trajectory(punch_traj.joint_trajectory)
        elif stage == 'punch':
            self.get_logger().info("Punch finished")
            self._pending_punch = None


# ---------------- RViz Panel ----------------
class ExecutePunchPanel(QWidget):
    def __init__(self, node: UR7ePunch):
        super().__init__()
        self.node = node
        self.layout = QVBoxLayout()

        self.preview_btn = QPushButton("Preview Punch")
        self.preview_btn.clicked.connect(self.preview_punch)
        self.layout.addWidget(self.preview_btn)

        self.confirm_btn = QPushButton("Confirm Punch")
        self.confirm_btn.clicked.connect(self.confirm_punch)
        self.layout.addWidget(self.confirm_btn)

        self.setLayout(self.layout)

        self.preview_cli = node.create_client(Trigger, '/execute_punch')
        self.confirm_cli = node.create_client(Trigger, '/confirm_punch')

    def preview_punch(self):
        req = Trigger.Request()
        if not self.preview_cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("Punch service not available")
            return
        future = self.preview_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result().success:
            self.node.get_logger().info("Punch preview published!")
        else:
            self.node.get_logger().error(f"Preview failed: {future.result().message}")

    def confirm_punch(self):
        req = Trigger.Request()
        if not self.confirm_cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("Confirm punch service not available")
            return
        future = self.confirm_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result().success:
            self.node.get_logger().info("Punch execution started!")
        else:
            self.node.get_logger().error(f"Punch execution failed: {future.result().message}")

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = UR7ePunch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
