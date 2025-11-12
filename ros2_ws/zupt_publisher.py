#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
import math

class ZUPTPub(Node):
    def __init__(self):
        super().__init__('zupt_publisher')
        self.sub = self.create_subscription(Imu, '/imu_converted', self.cb, 100)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/zupt/twist', 10)

        # 파라미터(필요시 조절)
        self.declare_parameter('gyro_thresh', 0.03)  # rad/s, 정지 판정 임계
        self.declare_parameter('cov_move', 1000.0)   # 이동 중: 크게
        self.declare_parameter('cov_stop', 0.0001)   # 정지 시: 아주 작게

        self.gyro_thresh = float(self.get_parameter('gyro_thresh').value)
        self.cov_move = float(self.get_parameter('cov_move').value)
        self.cov_stop = float(self.get_parameter('cov_stop').value)

    def cb(self, msg: Imu):
        gz = abs(msg.angular_velocity.z)
        moving = gz > self.gyro_thresh

        t = TwistWithCovarianceStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'

        # 정지로 가정한 관측값
        t.twist.twist.linear.x = 0.0
        t.twist.twist.linear.y = 0.0
        t.twist.twist.angular.z = 0.0

        cov = self.cov_move if moving else self.cov_stop
        # 6x6 공분산(flat): vx(0), vy(7), wz(35)을 세팅
        c = [0.0]*36
        c[0]  = cov    # vx
        c[7]  = cov    # vy
        c[35] = cov    # wz
        t.twist.covariance = c

        self.pub.publish(t)

def main():
    rclpy.init()
    node = ZUPTPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
