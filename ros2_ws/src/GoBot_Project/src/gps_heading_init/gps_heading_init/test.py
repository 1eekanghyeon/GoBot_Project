#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from rclpy.qos import qos_profile_sensor_data

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ScanSafetyGate(Node):
    """
    /scan + /cmd_vel_raw -> /cmd_vel_out

    - 전방 특정 각도 안에 stop_distance 미만 장애물이 있으면:
        => /cmd_vel_out = 0,0
    - 아니면:
        => /cmd_vel_raw 를 그대로 /cmd_vel_out 으로 패스
    """

    def __init__(self):
        super().__init__('scan_safety_gate')

        # ---- 파라미터 ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_out')
        self.declare_parameter('front_angle_deg', 60.0)  # 전방 ±30도
        self.declare_parameter('stop_distance', 1.0)     # 1m 이내면 정지

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_in_topic = self.get_parameter('cmd_in_topic').get_parameter_value().string_value
        self.cmd_out_topic = self.get_parameter('cmd_out_topic').get_parameter_value().string_value

        self.front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self.stop_dist = float(self.get_parameter('stop_distance').value)

        self.latest_scan = None
        self.blocked = False

        # ---- 서브스크립션/퍼블리셔 ----
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data,   # <-- LiDAR 쪽이랑 맞춰줌
        )

        self.sub_cmd = self.create_subscription(
            Twist, self.cmd_in_topic, self.cmd_cb, 10
        )
        self.pub_cmd = self.create_publisher(Twist, self.cmd_out_topic, 10)

        self.get_logger().info(
            f'ScanSafetyGate started: '
            f'scan={self.scan_topic}, in={self.cmd_in_topic}, out={self.cmd_out_topic}, '
            f'front=±{math.degrees(self.front_angle):.1f}deg, stop={self.stop_dist:.2f}m'
        )

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        min_r = float('inf')
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = angle_min + i * angle_inc
            # 전방 sector: -front_angle ~ +front_angle
            if abs(angle) <= self.front_angle:
                if r < min_r:
                    min_r = r

        if min_r < self.stop_dist:
            if not self.blocked:
                self.get_logger().info(
                    f'Obstacle detected at {min_r:.2f}m -> BLOCK'
                )
            self.blocked = True
        else:
            if self.blocked:
                self.get_logger().info(
                    f'Obstacle cleared (min_r={min_r:.2f}m) -> UNBLOCK'
                )
            self.blocked = False

    def cmd_cb(self, msg_in: Twist):
        # 기본값은 입력 그대로 복사
        cmd = Twist()
        cmd.linear.x = msg_in.linear.x
        cmd.linear.y = msg_in.linear.y
        cmd.linear.z = msg_in.linear.z
        cmd.angular.x = msg_in.angular.x
        cmd.angular.y = msg_in.angular.y
        cmd.angular.z = msg_in.angular.z

        # 스캔이 아직 없으면 그냥 통과
        if self.latest_scan is None:
            self.pub_cmd.publish(cmd)
            return

        # 막힌 상태면 강제 STOP
        if self.blocked:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
