# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, TransformStamped 
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

from planning.ik import IKPlanner

class UR7e_FollowPose(Node):
    TRANS_THRESHOLD = 0.05 # In meters
    THETA_THRESHOLD = 5 # In degrees

    def __init__(self):
        super().__init__('follow_pose')
        
        self.transform_sub = self.create_subscription(TransformStamped, '/body_transform', self.transform_callback, 1)
        self.hand_sub = self.create_subscription(Bool, '/hand_open', self.hand_callback, 1)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.current_transform = None
        self.joint_state = None
        self.current_hand = True # Gripper starts as open
        self.hand_changed = False

        self.ik_planner = IKPlanner()
        
        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

    def extract_rotation_and_translation(self, transform):
        quat = [transform.rotation.x, 
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w]

        translation = [transform.translation.x,
                       transform.translation.y,
                       transform.translation.z]

        rotation = R.from_quat(quat).as_matrix()

        return rotation, translation, quat
    
    def compute_transform_difference(self, trans1, trans2):
        rot1, trans1, _ = self.extract_rotation_and_translation(trans1)
        rot2, trans2, _ = self.extract_rotation_and_translation(trans2)
        # translation
        trans_err = np.linalg.norm(t2 - t1)
        # relative rotation
        rot_rel = rot1.T @ rot2
        # compute angle robustly
        cos_theta = clamp((np.trace(rot_rel) - 1.0) / 2.0, -1.0, 1.0)
        theta = np.arccos(cos_theta)  # radians
        theta = np.degrees(theta)
        return trans_err, theta

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def hand_callback(self, msg: Bool):
        self.hand_changed = msg.val != self.current_hand

    def transform_callback(self, msg: TransformStamped):
        trans_err, theta = self.compute_transform_different(self.current_transform, msg.transform)
        if trans_err <= self.TRANS_THRESHOLD and theta <= self.THETA_THRESHOLD:
            self.get_logger().info(f"Ttrans error: {trans_err}, Theta: {theta} -- Too similar to current pose!")
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return
        
        _, trans, quat = self.extract_rotation_and_translation(msg.transform)
        new_state = self.ik_planner.compute_ik(self.joint_state, *trans, *quat)
        self.job_queue.append(new_state)

        if self.hand_changed:
            self.job_queue.append("toggle_gripper")
            self.current_hand = not self.current_hand
        
        self.current_transform = msg.transform

        self.execute_jobs()


    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):

            traj = self.ik_planner.plan_to_joints(next_job)
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                return

            self.get_logger().info("Planned to position")

            self._execute_joint_trajectory(traj.joint_trajectory)
        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  # Proceed to next job

            
    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('bonk')
            rclpy.shutdown()
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_FollowPose()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
