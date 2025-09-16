#!/usr/bin/env python3
"""
Spawn obstacle script compatible with ROS2 Jazzy using ros_gz (preferred) 
and falling back to gazebo_ros.

Usage:
    ros2 run tb3_dynamic_nav_jazzy spawn_obstacle.py \
        --file ~/tb3_ws/src/tb3_dynamic_nav_jazzy/models/box/model.sdf \
        --name box1 --x 1.0 --y 0.0 --z 0.0
"""

import rclpy
from rclpy.node import Node
import argparse
import math
import os

from geometry_msgs.msg import Pose, Point, Quaternion

# Try ros_gz first
try:
    from ros_gz_interfaces.srv import SpawnEntity as RosGzSpawn
    HAVE_ROS_GZ = True
except ImportError:
    RosGzSpawn = None
    HAVE_ROS_GZ = False

# Fallback gazebo_ros
try:
    from gazebo_msgs.srv import SpawnEntity as GazeboSpawn
    HAVE_GAZEBO = True
except ImportError:
    GazeboSpawn = None
    HAVE_GAZEBO = False


def make_pose(x, y, z, yaw=0.0):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    p.orientation = q
    return p


def read_file(path):
    with open(path, 'r') as f:
        return f.read()


class Spawner(Node):
    def __init__(self, args):
        super().__init__('spawn_obstacle')
        self.args = args

        sdf = read_file(args.file)

        if HAVE_ROS_GZ:
            self.get_logger().info("Using ros_gz to spawn entity")
            self.client = self.create_client(RosGzSpawn, '/spawn_entity')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /spawn_entity (ros_gz)...')

            req = RosGzSpawn.Request()
            req.entity_factory.name = args.name
            req.entity_factory.sdf = sdf
            req.entity_factory.pose = make_pose(args.x, args.y, args.z)
            req.entity_factory.relative_to = 'world'
            self.future = self.client.call_async(req)

        elif HAVE_GAZEBO:
            self.get_logger().info("Using gazebo_ros to spawn entity")
            self.client = self.create_client(GazeboSpawn, '/spawn_entity')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /spawn_entity (gazebo_ros)...')

            req = GazeboSpawn.Request()
            req.name = args.name
            req.xml = sdf
            req.robot_namespace = ''
            req.initial_pose = make_pose(args.x, args.y, args.z)
            req.reference_frame = 'world'
            self.future = self.client.call_async(req)
        else:
            raise RuntimeError("Neither ros_gz nor gazebo_ros spawn services available")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True, help='Path to SDF/URDF file')
    parser.add_argument('--name', required=True, help='Name of the model')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Spawner(args)

    rclpy.spin_until_future_complete(node, node.future)
    if node.future.result() is not None:
        node.get_logger().info(f"Spawned entity: {args.name}")
    else:
        node.get_logger().error("Failed to spawn entity")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

