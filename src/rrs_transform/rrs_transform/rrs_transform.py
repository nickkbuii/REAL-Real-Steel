#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException

class RRSTransformer(Node):
    def __init__(self, body_base_link_frame, body_tool_frame):
        super().__init__('rrs_transformer')

        self.body_base_link_frame = body_base_link_frame
        self.body_tool_frame = body_tool_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pub = self.create_publisher(TransformStamped, 'body_transform', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f"RRSTransformer: Body Base Link Frame (torso)='{self.body_base_link_frame}', Body Tool Frame (hand)='{self.body_tool_frame}'"
        )

    def loop(self):
        try:
            msg = self.tf_buffer.lookup_transform(self.body_tool_frame, self.body_base_link_frame, Time())
            self.pub.publish(msg)

        except (TransformException, LookupException, ConnectivityException, ExtrapolationException):
            self.pub.publish(TransformStamped())
    
    def destroy_node(self):
        try:
            self.pub.publish(TransformStamped())
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: python3 rrs_transform.py body_base_link_frame body_tool_frame")
        rclpy.shutdown()
        return

    body_base_link_frame = sys.argv[1]
    body_tool_frame = sys.argv[2]

    node = RRSTransformer(body_base_link_frame, body_tool_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
