#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math

class VisualServoPointer(Node):
    def __init__(self):
        super().__init__('visual_servo_pointer')

        self.declare_parameter('pose_topic', '/aruco/map_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('xy_gain', 1.0)
        self.declare_parameter('stop_tolerance', 0.01)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.xy_gain = self.get_parameter('xy_gain').value
        self.stop_tolerance = self.get_parameter('stop_tolerance').value

        self.pointer_offset = 0.25
        self.max_speed = 0.25
        self.pose_buffer = []
        self.pose_buffer_size = 10
        self.target_pose = None
        self.reached_target = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.done_pub = self.create_publisher(String, '/visual_servo_done', 10)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)

        self.get_logger().info(f"Visual Servo started on topic: {self.pose_topic}")
        self.start_time = time.time()


    def _pose_cb(self, msg: PoseStamped):
        if self.reached_target:
            return

        if self.target_pose is None:
            self.pose_buffer.append((msg.pose.position.x, msg.pose.position.y))
            if len(self.pose_buffer) < self.pose_buffer_size:
                self.get_logger().info(f"Collecting poses... ({len(self.pose_buffer)}/{self.pose_buffer_size})")
                return
            avg_x = sum(p[0] for p in self.pose_buffer) / self.pose_buffer_size
            avg_y = sum(p[1] for p in self.pose_buffer) / self.pose_buffer_size
            self.target_pose = (avg_x, avg_y)
            self.get_logger().info(f"Target pose locked: x={avg_x:.2f}, y={avg_y:.2f}")
        
        if time.time() - self.start_time > 10.0:
            self.get_logger().warn("Timeout reached. Cancelling servoing.")
            self.cmd_pub.publish(Twist())
            self.reached_target = True
            self.done_pub.publish(String(data="failed"))
            return

        x, y = self.target_pose
        distance = math.sqrt(x**2 + y**2)
        move_distance = distance - self.pointer_offset

        if abs(move_distance) < self.stop_tolerance or move_distance < 0:
            self.get_logger().info("Reached target. Stopping servoing.")
            self.cmd_pub.publish(Twist())
            self.reached_target = True

            msg = String()
            msg.data = "done"
            self.done_pub.publish(msg)
            return

        norm = math.sqrt(x**2 + y**2)
        x_norm = x / norm
        y_norm = y / norm

        vx = self.xy_gain * x_norm * move_distance
        vy = self.xy_gain * y_norm * move_distance

        vx = max(min(vx, self.max_speed), -self.max_speed)
        vy = max(min(vy, self.max_speed), -self.max_speed)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"🔄 Moving → x={x:.2f}, y={y:.2f}, move={move_distance:.3f}, vx={vx:.2f}, vy={vy:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoPointer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
