#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from go2_interfaces.msg import IMU as Go2IMU


class IMUBridge(Node):
    """
    Go2 전용 IMU(go2_interfaces/IMU) → 표준 sensor_msgs/Imu 변환 브릿지
    - Quaternion, gyro(rad/s), accel(m/s^2) 매핑
    - 공분산(covariance) 대각 성분은 실측 분산 사용 + yaw/pitch 과신 방지 하한(floor)
    - frame_id/입출력 토픽은 파라미터로 설정 가능
    - QoS: sensor_data
    """

    def __init__(self):
        super().__init__('imu_bridge')

        # -------- Parameters (편의) --------
        self.declare_parameter('in_topic', '/imu')
        self.declare_parameter('out_topic', '/imu_converted')
        self.declare_parameter('frame_id', 'base_link')

        in_topic  = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.sub = self.create_subscription(
            Go2IMU, in_topic, self.callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.get_logger().info(f"✅ IMU bridge started: {in_topic} → {out_topic} (frame_id={self.frame_id})")

        # ======== 실측(정지 약 60s) 분산 결과 ========
        # Euler (rad) 분산
        self.roll_var_meas  = 6.246234e-04
        self.pitch_var_meas = 6.780636e-07
        self.yaw_var_meas   = 1.158429e-07
        # Gyro (rad/s)^2 분산
        self.gx_var = 4.833711e-05
        self.gy_var = 1.198881e-04
        self.gz_var = 1.108563e-04
        # Accel (m/s^2)^2 분산
        self.ax_var = 2.498673e-03
        self.ay_var = 1.512924e-03
        self.az_var = 1.155919e-03

        # 방향(orientation) 분산 하한 (과신 방지용)
        self.orient_var_floor = 1.0e-4  # ≈ 0.57° 표준편차

    def _floor_var(self, v):
        return v if v >= self.orient_var_floor else self.orient_var_floor

    def callback(self, msg: Go2IMU):
        imu_msg = Imu()

        # Header
        header = Header()
        # 가능하면 원본 타임스탬프 사용 (필드 없으면 로컬 clock 사용)
        stamp = None
        # go2_interfaces/IMU에 header나 time류 필드가 있다면 아래 try 블록 확장
        try:
            # 예) stamp = msg.header.stamp
            pass
        except Exception:
            pass
        try:
            header.stamp = msg.header.stamp   # Go2IMU에 header가 있으면 그대로
        except Exception:
            header.stamp = self.get_clock().now().to_msg()        
        header.frame_id = self.frame_id
        imu_msg.header = header

        # Orientation (quaternion)  [x, y, z, w] 가정
        imu_msg.orientation.x = float(msg.quaternion[0])
        imu_msg.orientation.y = float(msg.quaternion[1])
        imu_msg.orientation.z = float(msg.quaternion[2])
        imu_msg.orientation.w = float(msg.quaternion[3])

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = float(msg.gyroscope[0])
        imu_msg.angular_velocity.y = float(msg.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.gyroscope[2])

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = float(msg.accelerometer[0])
        imu_msg.linear_acceleration.y = float(msg.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.accelerometer[2])

        # Covariance (대각성분)
        imu_msg.orientation_covariance[0] = self._floor_var(self.roll_var_meas)   # roll
        imu_msg.orientation_covariance[4] = self._floor_var(self.pitch_var_meas)  # pitch
        imu_msg.orientation_covariance[8] = self._floor_var(self.yaw_var_meas)    # yaw

        imu_msg.angular_velocity_covariance[0] = self.gx_var
        imu_msg.angular_velocity_covariance[4] = self.gy_var
        imu_msg.angular_velocity_covariance[8] = self.gz_var

        imu_msg.linear_acceleration_covariance[0] = self.ax_var
        imu_msg.linear_acceleration_covariance[4] = self.ay_var
        imu_msg.linear_acceleration_covariance[8] = self.az_var

        self.pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
