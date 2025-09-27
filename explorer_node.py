#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from collections import deque
from std_msgs.msg import String

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        self.get_logger().info("Autonomous Explorer Node Activated")
        self.navigation_complete = False
        self.nav_goal_handle = None
        self.marker_target_set = False
        self.map_matrix = None
        self.map_metadata = None
        self.unsuccessful_goals = set()
        self.active_goal = None

        self.goal_input_sub = self.create_subscription(PoseStamped, '/set_goal', self.marker_input_callback, 10)
        self.map_feed_sub = self.create_subscription(OccupancyGrid, '/map', self.process_map, 10)
        self.servo_done_sub = self.create_subscription(String, '/visual_servo_done', self.visual_servo_complete_callback, 10)

        self.marker_engage_pub = self.create_publisher(String, '/start_marker_following', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.initiate_exploration()

    def process_map(self, msg):
        self.map_matrix = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info
        self.get_logger().info("🗺️ Map updated.")

    def marker_input_callback(self, msg):
        if self.marker_target_set:
            return

        self.get_logger().info("Marker goal received. Switching to marker pursuit mode.")
        self.marker_target_set = True

        if self.nav_goal_handle:
            self.nav_goal_handle.cancel_goal_async()
            self.get_logger().info("🛑 Previous navigation goal cancelled.")

        self.transmit_goal(msg)

    def is_exploration_edge(self, x, y):
        if self.map_matrix[y, x] != 0:
            return False
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map_matrix.shape[1] and 0 <= ny < self.map_matrix.shape[0]:
                    if self.map_matrix[ny, nx] == -1:
                        return True
        return False

    def expand_edge_cluster(self, sx, sy, visited):
        cluster = []
        q = deque()
        q.append((sx, sy))
        visited[sy, sx] = True

        while q:
            x, y = q.popleft()
            cluster.append((x, y))
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.map_matrix.shape[1] and 0 <= ny < self.map_matrix.shape[0] and
                        not visited[ny, nx] and self.is_exploration_edge(nx, ny)):
                        visited[ny, nx] = True
                        q.append((nx, ny))
        return cluster

    def locate_exploration_regions(self):
        if self.map_matrix is None:
            self.get_logger().warn("No map data yet.")
            return []

        exploration_regions = []
        visited = np.zeros_like(self.map_matrix, dtype=bool)

        for y in range(1, self.map_matrix.shape[0] - 1):
            for x in range(1, self.map_matrix.shape[1] - 1):
                if self.is_exploration_edge(x, y) and not visited[y, x]:
                    cluster = self.expand_edge_cluster(x, y, visited)
                    if cluster:
                        cx = sum(p[0] for p in cluster) / len(cluster)
                        cy = sum(p[1] for p in cluster) / len(cluster)
                        exploration_regions.append((cy, cx))

        self.get_logger().info(f"🔍 {len(exploration_regions)} unexplored regions detected.")
        return exploration_regions

    def exploration_region_to_pose(self, centroid):
        y, x = centroid
        res = self.map_metadata.resolution
        ox = self.map_metadata.origin.position.x
        oy = self.map_metadata.origin.position.y
        wx = x * res + ox
        wy = y * res + oy

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0
        return pose

    def transmit_goal(self, pose):
        self.active_goal = pose
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(goal)
        future.add_done_callback(self.handle_goal_response)

    def handle_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected.")
            self.initiate_exploration()
            return

        self.get_logger().info("Goal accepted. Navigating...")
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_goal_result)

    def handle_goal_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached goal.")
            if self.marker_target_set:
                self.get_logger().info("Switching to marker interaction.")
                msg = String()
                msg.data = "start"
                self.marker_engage_pub.publish(msg)
                return
        else:
            self.get_logger().warn("Failed to reach goal.")
            if self.active_goal:
                self.unsuccessful_goals.add((
                    round(self.active_goal.pose.position.x, 2),
                    round(self.active_goal.pose.position.y, 2)
                ))

        self.initiate_exploration()

    def visual_servo_complete_callback(self, msg: String):
        if msg.data == "done":
            self.get_logger().info("🔄 Visual servoing complete. Resuming exploration.")
        elif msg.data == "failed":
            self.get_logger().warn("❌ Servoing failed. Resuming exploration.")
        self.marker_target_set = False
        self.initiate_exploration()

    def initiate_exploration(self):
        if self.marker_target_set:
            self.get_logger().info("Exploration halted due to marker detection.")
            return

        regions = self.locate_exploration_regions()
        for region in regions:
            pose = self.exploration_region_to_pose(region)
            key = (
                round(pose.pose.position.x, 2),
                round(pose.pose.position.y, 2)
            )
            if key not in self.unsuccessful_goals:
                self.transmit_goal(pose)
                return

        self.get_logger().info("No unexplored regions currently. Retrying shortly.")
        self.create_timer(3.0, self.initiate_exploration)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
