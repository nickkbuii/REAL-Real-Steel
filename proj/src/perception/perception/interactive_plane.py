#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.spatial.transform import Rotation as R


class DraggableFistNode(Node):
    def __init__(self):
        super().__init__("draggable_fist_node")

        # Use base_link by default
        self.declare_parameter("frame_id", "base_link")

        frame_id = self.get_parameter("frame_id").value

        self.pose_pub = self.create_publisher(PoseStamped, "draggable_fist/pose", 10)
        self.create_timer(0.01, self.pub_pose)
        self.create_timer(0.1, self.compute_punch)

        self.server = InteractiveMarkerServer(self, "draggable_fist")

        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = frame_id
        self.int_marker.name = "fist"
        self.int_marker.description = "Punch target"
        self.int_marker.scale = 0.3

        # arrow marker
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = 0.3
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        vis_ctrl = InteractiveMarkerControl()
        vis_ctrl.always_visible = True
        vis_ctrl.markers.append(arrow)
        self.int_marker.controls.append(vis_ctrl)

        # movement
        self.add_axis_control(self.int_marker, "move_x", InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0)
        self.add_axis_control(self.int_marker, "move_y", InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0)
        self.add_axis_control(self.int_marker, "move_z", InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0)

        # === Rotation controls around each axis ===
        self.add_axis_control(self.int_marker, "rotate_x", InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0)
        self.add_axis_control(self.int_marker, "rotate_y", InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0)
        self.add_axis_control(self.int_marker, "rotate_z", InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0)
        self.server.insert(self.int_marker, feedback_callback=self.feedback_cb)
        self.server.applyChanges()

        self.get_logger().info("Punch marker in base_link ready!")

    def add_axis_control(self, marker, name, mode, x, y, z):
        ctrl = InteractiveMarkerControl()
        ctrl.name = name
        ctrl.interaction_mode = mode
        ctrl.orientation.w = 1.0
        ctrl.orientation.x = x
        ctrl.orientation.y = y
        ctrl.orientation.z = z
        marker.controls.append(ctrl)

    def feedback_cb(self, feedback):
        if feedback.event_type in (InteractiveMarkerFeedback.POSE_UPDATE,
                                   InteractiveMarkerFeedback.MOUSE_UP):
            msg = PoseStamped()
            msg.header = feedback.header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = feedback.pose
            self.pose_pub.publish(msg)

    def pub_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = self.int_marker.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self.int_marker.pose
        self.pose_pub.publish(msg)

    def compute_punch(self):
        pose = self.int_marker.pose

        position = np.array([pose.position.x,
                             pose.position.y,
                             pose.position.z])

        quat = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w]

        rot = R.from_quat(quat).as_matrix()
        punch_direction = rot @ np.array([0,0,1])

        self.get_logger().info(
            f"Punch target: {position}, direction: {punch_direction}"
        )


def main():
    rclpy.init()
    node = DraggableFistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
