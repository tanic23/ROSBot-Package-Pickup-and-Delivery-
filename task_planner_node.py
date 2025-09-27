#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
import heapq
import math

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # Marker pairs: pickup → delivery (even → odd)
        self.marker_pairs = {i: i + 1 for i in range(0, 99, 2)}

        # Robot task state
        self.carrying_list    = []  # list of (pickup_id, delivery_id)
        self.last_marker_id   = None
        self.waiting_for_nav  = False
        self.delivery_pose_buffer = {}  # delivery_id → PoseStamped

        # Pickup priority queue
        self.pickup_queue    = []
        self.MAX_QUEUE_SIZE  = 5

        # ROS interfaces
        self.create_subscription(Int32,       '/aruco/id',       self.on_marker_seen, 10)
        self.create_subscription(PoseStamped, '/aruco/map_pose', self.on_marker_pose, 10)

        self.goal_pub   = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        self.task_pub   = self.create_publisher(String,      '/task_command',     10)
        self.status_pub = self.create_publisher(String,      '/task_status',      10)

        self.get_logger().info("🚦 TaskPlannerNode started (map pose + delivery buffer)")
        self.publish_status("Idle: waiting for markers")

        # Will hold the rclpy Time when the last ID was seen
        self._last_seen_stamp = None

    def on_marker_seen(self, msg: Int32):
        # Store the ID and the exact time we saw it
        self.last_marker_id   = msg.data
        self._last_seen_stamp = self.get_clock().now()
        self.get_logger().info(f"Detected Marker ID: {msg.data}")

    def on_marker_pose(self, msg: PoseStamped):
        # 1️Drop if we haven't got an ID yet
        if self.last_marker_id is None:
            self.get_logger().warning("⚠ Pose received but no marker ID is currently set.")
            return

        # 2️Convert the incoming stamp into an rclpy Time for comparison
        pose_time = Time.from_msg(msg.header.stamp)

        # # 3️Drop if the pose is too far from when we saw the ID
        # if self._last_seen_stamp is None \
        #    or abs((pose_time - self._last_seen_stamp).nanoseconds) > 1e8:
        #     self.get_logger().warning("⚠ Pose timestamp doesn't match latest ID, skipping")
        #     return

        # 4️We have a matched ID + pose — pull out the values
        mid   = self.last_marker_id
        x     = msg.pose.position.x
        y     = msg.pose.position.y
        stamp = msg.header.stamp

        # Reset for the next marker
        self.last_marker_id   = None
        self._last_seen_stamp = None

        # Delivery Marker?
        if mid % 2 == 1:
            self.delivery_pose_buffer[mid] = msg
            for pair in list(self.carrying_list):
                if mid == pair[1]:
                    self.get_logger().info(f"Navigating to Delivery Marker {mid} (for pickup {pair[0]})")
                    self.send_goal(x, y, stamp)
                    self.send_task(f"delivery:{mid}")
                    self.publish_status(f"Delivering to marker {mid}")
                    self.carrying_list.remove(pair)
                    self.waiting_for_nav = False
                    self.get_logger().info(f"elivery Complete: {pair[0]} → {mid}")
                    return
            self.get_logger().info(f"Delivery pose {mid} cached for future use.")
            return

        # Pickup Marker?
        if mid in self.marker_pairs:
            dist = math.hypot(x, y)
            if len(self.pickup_queue) < self.MAX_QUEUE_SIZE:
                delivery_id = self.marker_pairs[mid]
                bonus = -1.0 if delivery_id in self.delivery_pose_buffer else 0.0
                heapq.heappush(self.pickup_queue, (dist + bonus, mid, delivery_id))
                self.get_logger().info(
                    f"Queued Pickup {mid} → Delivery {delivery_id} (score {dist + bonus:.2f}"
                )
                self.try_next_task()
            else:
                self.get_logger().warning("⚠ Pickup queue full. Marker ignored.")

    def try_next_task(self):
        if self.waiting_for_nav or not self.pickup_queue:
            return

        _, pickup_id, delivery_id = heapq.heappop(self.pickup_queue)
        self.get_logger().info(f"Starting Task: Pickup {pickup_id} → Delivery {delivery_id}")

        self.last_marker_id = pickup_id
        self.carrying_list.append((pickup_id, delivery_id))
        self.publish_status(f"Waiting to pick up from marker {pickup_id}")
        self.waiting_for_nav = True

    def send_goal(self, x, y, stamp):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp    = stamp
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Navigation Goal Sent: x={x:.2f}, y={y:.2f}")

    def send_task(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.task_pub.publish(msg)
        self.get_logger().info(f"Task Command Published: '{cmd}'")

    def publish_status(self, status: str):
        full_status = status + " | Carrying: " + str(self.carrying_list)
        msg = String()
        msg.data = full_status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status Update: {full_status}")

def main(args=None)
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()