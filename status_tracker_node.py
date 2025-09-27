import rclpy
from std_msgs.msg import Int32, String, Bool
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from datetime import datetime
import os
import csv

class StatusTracker(Node):
    def __init__(self):
        super().__init__('status_tracker_node')

        # Subscribers
        self.create_subscription(String, '/task_status', self.task_status_callback, 10)
        self.create_subscription(PoseStamped, '/aruco/map_pose', self.marker_callback, 10)
        self.create_subscription(Bool, '/pointer/aligned', self.pointer_callback, 10)
        self.create_subscription(Int32, '/aruco/id', self.id_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/package_status', 10)
        self.complete_pub = self.create_publisher(Bool, '/delivery_complete', 10)
        self.log_pub = self.create_publisher(String, '/log_events', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/task_visualization', 10)

        # State
        self.expected_marker_id = None
        self.expected_task_type = None
        self.last_seen_marker_id = None
        self.current_marker_pose = None
        self.marker_history = []

        # CSV file for logs
        self.log_file_path = os.path.join(os.getcwd(), 'delivery_log.csv')
        if not os.path.exists(self.log_file_path):
            with open(self.log_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Task Type', 'Marker ID'])

        self._last_aruco_id = None

        self.get_logger().info("✅ Status Tracker Node (ROS 2) is running...")

    def task_status_callback(self, msg):
        task_text = msg.data.lower()
        self.get_logger().info(f"[TaskStatus] {task_text}")
        self.write_log_text(f"[TaskStatus] {task_text}")

        if "picking up from marker" in task_text:
            self.expected_task_type = "pickup"
            self.expected_marker_id = int(task_text.split("marker")[1].strip())
        elif "delivering to marker" in task_text:
            self.expected_task_type = "delivery"
            self.expected_marker_id = int(task_text.split("marker")[1].strip())
        else:
            self.expected_marker_id = None
            self.expected_task_type = None

    def id_callback(self, msg: Int32):
        self._last_aruco_id = msg.data

    def marker_callback(self, msg: PoseStamped):
        if self._last_aruco_id is None:
            return
        self.last_seen_marker_id = self._last_aruco_id
        self.current_marker_pose = msg.pose
        self._last_aruco_id = None

    def pointer_callback(self, msg):
        self.get_logger().info(f"[DEBUG] pointer/aligned = {msg.data}, expected_marker_id = {self.expected_marker_id}, last_seen = {self.last_seen_marker_id}")
        if msg.data and self.expected_marker_id is not None and self.current_marker_pose is not None:
            if self.last_seen_marker_id == self.expected_marker_id:
                self.log_task_completion()

    def log_task_completion(self):
        now = datetime.now()
        time_str = now.strftime('%Y-%m-%d %H:%M:%S')
        status_str = f"{self.expected_task_type.capitalize()} completed for Marker {self.expected_marker_id} at {time_str}"

        self.get_logger().info(f"[STATUS] {status_str}")
        self.status_pub.publish(String(data=status_str))
        self.complete_pub.publish(Bool(data=True))
        self.log_pub.publish(String(data=status_str))
        self.write_log_csv(time_str, self.expected_task_type, self.expected_marker_id)
        self.publish_marker(self.expected_task_type, self.expected_marker_id, self.current_marker_pose)

        # Reset for next task
        self.expected_marker_id = None
        self.expected_task_type = None
        self.current_marker_pose = None
        self.last_seen_marker_id = None

    def write_log_text(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f"[LOG] {message}")

    def write_log_csv(self, timestamp, task_type, marker_id):
        with open(self.log_file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, task_type, marker_id])

    def publish_marker(self, task_type, marker_id, pose):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "task_history"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        if task_type == "pickup":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_history.append(marker)
        ma = MarkerArray()
        ma.markers = self.marker_history
        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = StatusTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
