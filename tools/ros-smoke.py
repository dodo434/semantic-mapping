#!/usr/bin/env python3
"""Exercise the installed Nav2 semantic plugin in a motor-free ROS graph."""

from array import array
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def main():
    rclpy.init()
    node = Node('semantic_nav_smoke')
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    navigation = node.create_publisher(Twist, '/semantic/nav_velocity', 1)
    publisher = node.create_publisher(OccupancyGrid, '/semantic/semantic_grid', 1)
    last_cost = None
    saw_stop = False

    def cost(message):
        nonlocal last_cost
        last_cost = message

    def velocity(message):
        nonlocal saw_stop
        if message.linear.x != 0 or message.angular.z != 0:
            raise AssertionError('Disabled velocity gate forwarded motion')
        saw_stop = True

    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', cost, 1)
    node.create_subscription(Twist, '/semantic/bench_velocity', velocity, 1)
    bt_state = node.create_client(GetState, '/bt_navigator/get_state')
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.05
    grid.info.width = grid.info.height = 80
    grid.info.origin.position.x = grid.info.origin.position.y = -2.0
    grid.info.origin.orientation.w = 1.0
    grid.data = array('b', [0] * 6400)
    # Place a solid obstacle away from the origin.
    grid.data[40*80+60] = 100
    command = Twist()
    command.linear.x = 0.2

    def spin_until(timeout, predicate, feed_map):
        limit = time.monotonic() + timeout
        while time.monotonic() < limit:
            if feed_map:
                grid.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(grid)
                navigation.publish(command)
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def fresh_cost():
        return (last_cost is not None and 0 in last_cost.data and 100 in last_cost.data
                and saw_stop)

    def stale_cost():
        return last_cost is not None and all(value == 100 for value in last_cost.data)

    try:
        if not spin_until(45.0, fresh_cost, True):
            raise RuntimeError('Nav2 never published a map with free and lethal cells')
        if not bt_state.wait_for_service(timeout_sec=5):
            raise RuntimeError('BT navigator lifecycle service is unavailable')
        future = bt_state.call_async(GetState.Request())
        if not spin_until(15.0, lambda: future.done(), True) or future.result().current_state.label != 'active':
            # Lifecycle transitions can still be in flight after the first costmap.
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                future = bt_state.call_async(GetState.Request())
                if spin_until(1.0, lambda: future.done(), True) and future.result().current_state.label == 'active':
                    break
            else:
                raise RuntimeError('BT navigator did not activate')
        if not spin_until(6.0, stale_cost, False):
            raise RuntimeError('Stale semantic input did not block the costmap')
        print('Nav2 loaded the plugin, marked stale data lethal and withheld base motion.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
