#!/usr/bin/env python3
import rclpy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'libs'))

from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ExtrapolationException,
    TransformException,
)
from transformations import quaternion_multiply, quaternion_matrix
import numpy as np


class TFPoseTransformer(Node):
    def __init__(self):
        super().__init__('tf_pose_transformer')

        self.declare_parameter('input_pose_topic', '/aruco/pose')
        self.declare_parameter('input_id_topic', '/aruco/id')
        self.declare_parameter('output_pose_topic', '/aruco/map_pose')
        self.declare_parameter('target_frame', 'map')

        self.in_pose = self.get_parameter('input_pose_topic').value
        self.in_id = self.get_parameter('input_id_topic').value
        self.out_pose = self.get_parameter('output_pose_topic').value
        self.tf_target = self.get_parameter('target_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_pub = self.create_publisher(PoseStamped, self.out_pose, 10)
        self.current_id = None

        self.create_subscription(Int32, self.in_id, self._id_cb, 10)
        self.create_subscription(PoseStamped, self.in_pose, self._pose_cb, 10)

        self.get_logger().info(
            f"TFPoseTransformer:\n"
            f"  → listening  : {self.in_pose}\n"
            f"  → id topic   : {self.in_id}\n"
            f"  → publishing : {self.out_pose} (frame `{self.tf_target}`)"
        )

    def _id_cb(self, msg: Int32):
        self.current_id = msg.data

    def _pose_cb(self, msg: PoseStamped):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.tf_target,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, ExtrapolationException, TransformException) as e:
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return

        tf_trans = tf_stamped.transform.translation
        tf_rot = tf_stamped.transform.rotation

        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]

        translation = np.array([tf_trans.x, tf_trans.y, tf_trans.z])
        rot_matrix = quaternion_matrix([tf_rot.x, tf_rot.y, tf_rot.z, tf_rot.w])
        local_point = np.array([px, py, pz, 1.0])
        transformed_point = rot_matrix @ local_point
        transformed_position = transformed_point[:3] + translation

        out_pose = PoseStamped()
        out_pose.header.stamp = self.get_clock().now().to_msg()
        out_pose.header.frame_id = self.tf_target
        out_pose.pose.position.x = transformed_position[0]
        out_pose.pose.position.y = transformed_position[1]
        out_pose.pose.position.z = transformed_position[2]

        out_q = quaternion_multiply(
            [tf_rot.x, tf_rot.y, tf_rot.z, tf_rot.w], q
        )
        out_pose.pose.orientation.x = out_q[0]
        out_pose.pose.orientation.y = out_q[1]
        out_pose.pose.orientation.z = out_q[2]
        out_pose.pose.orientation.w = out_q[3]

        self.map_pub.publish(out_pose)

        if self.current_id is not None:
            self.get_logger().info(
                f"Transformed marker_{self.current_id} to `{self.tf_target}` frame "
                f"at x={out_pose.pose.position.x:.2f}, y={out_pose.pose.position.y:.2f}, z={out_pose.pose.position.z:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TFPoseTransformer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()