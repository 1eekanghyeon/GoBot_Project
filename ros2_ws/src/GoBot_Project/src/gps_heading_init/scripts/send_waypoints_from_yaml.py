#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints
import yaml, sys, time

class WaypointSender(Node):
    def __init__(self, poses):
        super().__init__('waypoint_sender')
        self.poses = poses
        self.cli_ntp = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.cli_fw  = ActionClient(self, FollowWaypoints,       'follow_waypoints')

    def send(self):
        # 1) try NavigateThroughPoses
        self.get_logger().info('Waiting for navigate_through_poses...')
        if self.cli_ntp.wait_for_server(timeout_sec=3.0):
            goal = NavigateThroughPoses.Goal()
            goal.poses = self.poses
            self.get_logger().info(f'Sending {len(self.poses)} poses via NavigateThroughPoses')
            send_future = self.cli_ntp.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info(f'NTP result: {result_future.result().result}')
            return True

        # 2) fallback: FollowWaypoints
        self.get_logger().warn('navigate_through_poses not available, trying follow_waypoints...')
        if self.cli_fw.wait_for_server(timeout_sec=5.0):
            goal = FollowWaypoints.Goal()
            goal.poses = self.poses
            self.get_logger().info(f'Sending {len(self.poses)} poses via FollowWaypoints')
            send_future = self.cli_fw.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info(f'FW result: {result_future.result().result}')
            return True

        self.get_logger().error('No waypoint action servers are available.')
        return False

def load_yaml_poses(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    seq = data.get('poses') or data.get('waypoints') or []
    poses = []
    for d in seq:
        ps = PoseStamped()
        ps.header.frame_id = (d.get('header') or {}).get('frame_id','map')
        p = d['pose']['position']; q = d['pose']['orientation']
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(p['x']), float(p['y']), float(p.get('z',0.0))
        ps.pose.orientation.x = float(q.get('x',0.0))
        ps.pose.orientation.y = float(q.get('y',0.0))
        ps.pose.orientation.z = float(q.get('z',0.0))
        ps.pose.orientation.w = float(q.get('w',1.0))
        poses.append(ps)
    if not poses:
        raise RuntimeError('No poses/waypoints found in YAML')
    return poses

def main():
    if len(sys.argv) != 2:
        print("Usage: send_waypoints_from_yaml.py <waypoints.yaml>")
        sys.exit(2)
    path = sys.argv[1]
    poses = load_yaml_poses(path)
    rclpy.init()
    node = WaypointSender(poses)
    ok = node.send()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()
