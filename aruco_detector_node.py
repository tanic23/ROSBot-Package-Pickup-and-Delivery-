#!/usr/bin/env python3
import rclpy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'libs'))

import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from transforms3d.quaternions import mat2quat
from tf2_ros import TransformBroadcaster, TransformStamped


class ArucoDetector(Node):
    """
    Subscribes to an image + camera_info, detects ArUco markers,
    publishes PoseStamped + Int32 ID, spherical markers, and broadcasts TF frames.
    """

    def __init__(self):
        super().__init__('aruco_detector')

        # ---- parameters ----
        self.declare_parameter('image_topic',       '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_size',        0.095)            # meters
        self.declare_parameter('aruco_dictionary',  'DICT_4X4_50')

        self.image_topic       = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size       = self.get_parameter('marker_size').value
        dict_name              = self.get_parameter('aruco_dictionary').value

        # ---- OpenCV / ArUco setup ----
        self.bridge        = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.aruco_dict    = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
        self.aruco_params  = aruco.DetectorParameters_create()

        # ---- TF broadcaster ----
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- publishers ----
        self.pose_pub    = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.id_pub      = self.create_publisher(Int32,       '/aruco/id',   10)
        self.marker_pub  = self.create_publisher(Marker,      '/aruco/marker', 10)

        # ---- subscriptions ----
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data)

        self.img_sub  = self.create_subscription(
            Image, self.image_topic,
            self._on_image,
            qos_profile_sensor_data)

        self.get_logger().info(
            '🕵️ ArUcoDetector ready\n'
            f'  image:       {self.image_topic}\n'
            f'  camera_info: {self.camera_info_topic}'
        )

    def _on_camera_info(self, msg: CameraInfo):
        """Grab camera intrinsics (once) and then unsubscribe."""
        self.camera_matrix = np.array(msg.k).reshape((3,3))
        self.dist_coeffs   = np.array(msg.d)
        self.get_logger().info('📐 Got camera intrinsics')
        self.destroy_subscription(self.info_sub)

    def _on_image(self, msg: Image):
        """Each frame: detect markers, publish pose+ID and broadcast TF."""
        if self.camera_matrix is None:
            return

        # Convert ROS→CV and grayscale
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        # (Optional) refine
        aruco.refineDetectedMarkers(
            image=gray,
            board=None,
            detectedCorners=corners,
            detectedIds=ids,
            rejectedCorners=None,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs)

        for corner, mid in zip(corners, ids.flatten()):
            # Estimate pose
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corner, self.marker_size,
                self.camera_matrix, self.dist_coeffs)

            # Build PoseStamped
            ps = PoseStamped()
            ps.header = msg.header
            # ps.header.frame_id = 'camera_link'
            ps.header.frame_id = 'oak_camera_rgb_camera_optical_frame'  # <- FIXED FRAME


            t = tvecs[0][0]
            ps.pose.position.x = float(t[0])
            ps.pose.position.y = float(t[1])
            ps.pose.position.z = float(t[2])

            # Rotation → quaternion [w,x,y,z]
            R, _ = cv2.Rodrigues(rvecs[0][0])
            w, x, y, z = mat2quat(R)
            ps.pose.orientation.w = w
            ps.pose.orientation.x = x
            ps.pose.orientation.y = y
            ps.pose.orientation.z = z

            # Publish Pose + ID
            self.pose_pub.publish(ps)
            self.id_pub.publish(Int32(data=int(mid)))

            # Broadcast TF: camera_link → marker_<ID>
            tf = TransformStamped()
            tf.header         = ps.header
            tf.child_frame_id = f'marker_{mid}'
            tf.transform.translation = Vector3(
                x=ps.pose.position.x,
                y=ps.pose.position.y,
                z=ps.pose.position.z
            )
            tf.transform.rotation = ps.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

            # Publish Sphere Marker
            marker = Marker()
            marker.header = ps.header
            marker.ns = "aruco"
            marker.id = int(mid)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = ps.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10  # ← Updated to 10cm sphere
            # marker.scale.x = 0.05
            # marker.scale.y = 0.05
            # marker.scale.z = 0.05
            # marker.color.r = 1.0
            # marker.color.g = 0.0
            # marker.color.b = 0.0
            # marker.color.a = 1.0
            if marker.id % 2 == 0:
                # Pickup: Red
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # Delivery: Green
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0).to_msg()  # persistent
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()