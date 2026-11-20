"""Bounded bench velocity gate. The base still needs its hardware watchdog."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Header, Int32


def limit(value, bound):
    return min(bound, max(-bound, value))


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        self.max_linear = self.declare_parameter('max_linear', 0.25).value
        self.max_angular = self.declare_parameter('max_angular', 0.7).value
        self.enabled = self.declare_parameter('enabled', False).value
        if not (0 < self.max_linear <= 1.0 and 0 < self.max_angular <= 2.0):
            raise ValueError('Velocity limits must be positive and within the bench bounds')
        self.arm_until = self.pose_until = self.grid_until = self.command_until = 0.0
        self.command = Twist()
        self.tracking = False
        self.revision = 0.0
        self.create_subscription(Bool, '/semantic/armed', self.arm, 1)
        self.create_subscription(PoseStamped, '/semantic/body_pose', self.pose, 1)
        self.create_subscription(OccupancyGrid, '/semantic/semantic_grid', self.grid, 1)
        self.create_subscription(Int32, '/semantic/tracking_state', self.state, 1)
        self.create_subscription(
            Header, '/semantic/map_revision', self.reset,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Twist, '/semantic/nav_velocity', self.receive, 1)
        self.output = self.create_publisher(Twist, 'base_command', 1)
        self.wall_timer = self.create_timer(0.05, self.publish)

    def arm(self, message):
        self.arm_until = time.monotonic() + 0.35 if message.data else 0.0

    @staticmethod
    def stamp(message):
        stamp = message.header.stamp if hasattr(message, 'header') else message.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def freshness(self, message, limit_sec):
        acquisition = self.stamp(message)
        if acquisition < self.revision:
            return 0.0
        age = self.get_clock().now().nanoseconds * 1e-9 - acquisition
        return time.monotonic() + limit_sec - max(age, 0) if -0.05 <= age < limit_sec else 0.0

    def pose(self, message):
        self.pose_until = self.freshness(message, 0.50)

    def grid(self, message):
        self.grid_until = self.freshness(message, 0.75)

    def state(self, message):
        self.tracking = message.data == 2  # ORB-SLAM3 Tracking::OK

    def reset(self, message):
        self.revision = self.stamp(message)
        self.pose_until = self.grid_until = 0.0

    def receive(self, message):
        x, z = message.linear.x, message.angular.z
        if not math.isfinite(x) or not math.isfinite(z):
            self.command_until = 0.0
            return
        self.command = Twist()
        self.command.linear.x = limit(x, self.max_linear)
        self.command.angular.z = limit(z, self.max_angular)
        self.command_until = time.monotonic() + 0.25

    def publish(self):
        healthy = (self.enabled and self.tracking and time.monotonic() <
                   min(self.arm_until, self.pose_until, self.grid_until, self.command_until))
        self.output.publish(self.command if healthy else Twist())


def main():
    rclpy.init()
    try:
        node = VelocityGate()
        try:
            rclpy.spin(node)
        finally:
            if node.context.ok():
                node.output.publish(Twist())
            node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
