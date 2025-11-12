#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class GPSHeadingNode(Node):
    """
    GPS 기반으로 ENU 헤딩을 만들어서 로봇의 부팅-기준 yaw에 덮어쓰는 노드.

    상태머신:
      - IDLE: start 서비스 안 온 상태
      - WAIT_FIX: GPS가 FIX 될 때까지 기다림
      - COLLECT_INITIAL: 최초 보정용 3m 세그먼트 수집 + (이때 자동 직진)
      - LOCKED: offset 확정, 이후 /imu/gps_heading 퍼블리시 + 재보정 모니터링
    """

    def __init__(self):
        super().__init__('gps_heading_init')

        # 상태 정의
        self.STATE_IDLE = 0
        self.STATE_WAIT_FIX = 1
        self.STATE_COLLECT_INITIAL = 2
        self.STATE_LOCKED = 3

        self.state = self.STATE_IDLE
        self.offset = 0.0  # 현재 적용 중인 yaw 오프셋 (gps_yaw - robot_yaw)

        # -------- 파라미터 --------
        # odom_topic: 실제 토픽은 /odom 이라서 기본값을 /odom 으로 둔다
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('imu_output_topic', '/imu/gps_heading')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_out')
        self.declare_parameter('test_speed', 0.3)        # m/s, 초기 직진 속도
        self.declare_parameter('max_initial_time', 10.0) # s, 3m 못 가도 이 시간 지나면 정지

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        imu_output_topic = self.get_parameter('imu_output_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.test_speed = float(self.get_parameter('test_speed').get_parameter_value().double_value)
        self.max_initial_time = float(self.get_parameter('max_initial_time').get_parameter_value().double_value)

        # QoS: GPS는 BEST_EFFORT, 나머지도 그냥 맞춰줌 (지금까지 잘 되던 설정 유지)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # -------- 구독 --------
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)
        self.create_subscription(NavSatFix, fix_topic, self.fix_callback, qos)

        # IMU에서 roll/pitch 복사용
        try:
            self.create_subscription(Imu, '/imu_converted', self.imu_callback, qos)
        except Exception:
            self.get_logger().info('No /imu_converted topic; roll/pitch will be zero')

        # -------- 퍼블리셔 --------
        self.imu_pub = self.create_publisher(Imu, imu_output_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # -------- 서비스 --------
        self.create_service(Trigger, 'gps_heading_init/start', self.start_callback)

        # -------- 초기 보정용 상태 변수들 --------
        self.lat1 = None
        self.lon1 = None
        self.prev_lat = None
        self.prev_lon = None
        self.heading_list = []
        self.distance_travelled = 0.0
        self.collect_start_time = None

        # -------- 재보정용 상태 변수들 --------
        self.last_lat = None
        self.last_lon = None
        self.recalc_heading_list = []
        self.recalc_distance = 0.0
        self.recalc_prev_east = 0.0
        self.recalc_prev_north = 0.0
        self.time_last_recalc = self.get_clock().now()

        # -------- 최신 로봇 자세 --------
        self.yaw_robot = None
        self.roll = 0.0
        self.pitch = 0.0

        # yaw 공분산 (rad^2). 처음엔 적당한 값으로 초기화
        self.yaw_variance = 0.001  # ≈ 1.8° 표준편차

        # cmd_vel 멈춤을 한 번만 보내기 위한 플래그
        self.need_stop_once = False

        # 10Hz 타이머: 자동 직진 / 정지 명령
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('gps_heading_init node started. Call /gps_heading_init/start to begin.')

    # ---------------- 타이머: 자동 직진 / 정지 ----------------
    def timer_callback(self):
        """
        - STATE_COLLECT_INITIAL 동안 test_speed 로 /cmd_vel_out 직진
        - 상태가 바뀌면서 need_stop_once=True 가 되면 0 명령 한 번만 내보내고 멈춤
          (Nav2 등 다른 노드가 /cmd_vel_out 쓰기 시작해도 방해 안 하도록)
        """
        if self.state == self.STATE_COLLECT_INITIAL:
            cmd = Twist()
            cmd.linear.x = self.test_speed
            self.cmd_pub.publish(cmd)
        elif self.need_stop_once:
            # 한 번만 STOP 보내고 플래그 끔
            cmd = Twist()  # 전부 0
            self.cmd_pub.publish(cmd)
            self.need_stop_once = False

    # ---------------- 서비스 콜백 ----------------
    def start_callback(self, request, response):
        if self.state != self.STATE_IDLE:
            response.success = False
            response.message = 'Already started.'
            return response

        self.state = self.STATE_WAIT_FIX
        self.get_logger().info('Start signal received. Waiting for GPS FIX...')
        response.success = True
        response.message = 'Waiting for GPS FIX'
        return response

    # ---------------- Odometry 콜백 ----------------
    def odom_callback(self, msg: Odometry):
        # 쿼터니언 -> yaw
        ori = msg.pose.pose.orientation
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.yaw_robot = math.atan2(siny, cosy)

        # LOCKED 상태에서는 바로 보정된 IMU 내보내기
        if self.state == self.STATE_LOCKED:
            yaw_corr = self.yaw_robot + self.offset
            # 정규화 [-pi, pi]
            yaw_corr = math.atan2(math.sin(yaw_corr), math.cos(yaw_corr))

            imu_msg = Imu()
            imu_msg.header.stamp = msg.header.stamp
            imu_msg.header.frame_id = 'base_link'

            # roll/pitch는 실제 IMU에서 가져온 값 사용 (없으면 0)
            cy = math.cos(yaw_corr * 0.5)
            sy = math.sin(yaw_corr * 0.5)
            cr = math.cos(self.roll * 0.5)
            sr = math.sin(self.roll * 0.5)
            cp = math.cos(self.pitch * 0.5)
            sp = math.sin(self.pitch * 0.5)

            # ZYX 순서로 쿼터니언 합성
            imu_msg.orientation.w = cy * cp * cr + sy * sp * sr
            imu_msg.orientation.x = cy * cp * sr - sy * sp * cr
            imu_msg.orientation.y = sy * cp * sr + cy * sp * cr
            imu_msg.orientation.z = sy * cp * cr - cy * sp * sr
            imu_msg.orientation_covariance = [
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  self.yaw_variance
            ]

            # 각속도/가속도는 제공 안 하므로 -1.0으로 '측정 안 함' 표기
            imu_msg.angular_velocity_covariance = [
                -1.0, 0.0, 0.0,
                 0.0, -1.0, 0.0,
                 0.0, 0.0, -1.0
            ]
            imu_msg.linear_acceleration_covariance = [
                -1.0, 0.0, 0.0,
                 0.0, -1.0, 0.0,
                 0.0, 0.0, -1.0
            ]
            self.imu_pub.publish(imu_msg)

    # ---------------- IMU 콜백 (roll/pitch 복사) ----------------
    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # roll
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.roll = math.atan2(sinr, cosr)

        # pitch
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            self.pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.pitch = math.asin(sinp)

    # ---------------- GPS 콜백 ----------------
    def fix_callback(self, msg: NavSatFix):
        # 시작 안 했으면 무시
        if self.state == self.STATE_IDLE:
            return

        # 쓸 수 없는 GPS면 무시
        if msg.status.status < 0:
            # COLLECT 중에 FIX 잃으면 다시 기다리기 + 정지
            if self.state == self.STATE_COLLECT_INITIAL:
                self.get_logger().warn('GPS FIX lost. Back to WAIT_FIX.')
                self.state = self.STATE_WAIT_FIX
                self.need_stop_once = True
            return

        lat = msg.latitude
        lon = msg.longitude

        # 1) FIX 기다리는 중
        if self.state == self.STATE_WAIT_FIX:
            # 첫 점 저장
            self.lat1 = lat
            self.lon1 = lon
            self.prev_lat = lat
            self.prev_lon = lon
            self.heading_list = []
            self.distance_travelled = 0.0
            self.collect_start_time = self.get_clock().now()
            self.need_stop_once = False  # 이제부터는 직진 시작

            self.get_logger().info(
                f'Initial GPS point stored ({lat:.6f}, {lon:.6f}). '
                f'Auto-driving ~3m straight with v={self.test_speed:.2f} m/s.'
            )
            self.state = self.STATE_COLLECT_INITIAL
            return

        # 2) 최초 3m 수집 중 (이때 timer_callback 이 계속 직진 명령 보냄)
        if self.state == self.STATE_COLLECT_INITIAL:
            # 경과 시간 체크 (실내에서 GPS 안 움직여도 무한히 달리지 않게)
            if self.collect_start_time is not None:
                elapsed = (self.get_clock().now() - self.collect_start_time).nanoseconds * 1e-9
            else:
                elapsed = 0.0

            dlat = math.radians(lat - self.lat1)
            dlon = math.radians(lon - self.lon1)
            east = 6378137.0 * dlon * math.cos(math.radians(self.lat1))
            north = 6378137.0 * dlat
            self.distance_travelled = math.hypot(east, north)

            # 한 스텝 방향 추정 (직진성 보려고)
            prev_dlat = math.radians(self.prev_lat - self.lat1)
            prev_dlon = math.radians(self.prev_lon - self.lon1)
            prev_east = 6378137.0 * prev_dlon * math.cos(math.radians(self.lat1))
            prev_north = 6378137.0 * prev_dlat

            step_east = east - prev_east
            step_north = north - prev_north
            if math.hypot(step_east, step_north) > 0.001:
                step_heading = math.degrees(math.atan2(step_north, step_east))
                self.heading_list.append(step_heading)

            self.prev_lat = lat
            self.prev_lon = lon

            # 안전장치: max_initial_time 안에 최소 1m도 못 가면 포기하고 멈춤
            if elapsed > self.max_initial_time and self.distance_travelled < 1.0:
                self.get_logger().warn(
                    f'Initial calibration timeout (t={elapsed:.1f}s, dist={self.distance_travelled:.2f}m). '
                    f'Back to WAIT_FIX.'
                )
                self.state = self.STATE_WAIT_FIX
                self.need_stop_once = True
                return

            # 길이 + 직진성 조건 체크
            if self.distance_travelled >= 3.0 and len(self.heading_list) >= 2:
                mean_h = sum(self.heading_list) / len(self.heading_list)
                var_h = sum((h - mean_h) ** 2 for h in self.heading_list) / len(self.heading_list)
                stddev_h = math.sqrt(var_h)

                if stddev_h <= 2.0:
                    # ENU 기준 yaw 계산
                    yaw_gps = math.atan2(north, east)  # ENU: x=E, y=N
                    if self.yaw_robot is None:
                        self.get_logger().error('No odom yaw yet; cannot compute offset.')
                        return
                    yaw_robot = self.yaw_robot
                    offset = yaw_gps - yaw_robot
                    offset = math.atan2(math.sin(offset), math.cos(offset))
                    self.offset = offset
                    
                    # --- 여기 추가: yaw 공분산 갱신 ---
                    stddev_rad = max(math.radians(stddev_h), math.radians(0.5))
                    self.yaw_variance = stddev_rad * stddev_rad

                    self.get_logger().info(
                        f'Initial calibration OK. dist={self.distance_travelled:.2f} m, std={stddev_h:.2f} deg')
                    self.get_logger().info(
                        f'GPS yaw={yaw_gps:.3f}, robot yaw={yaw_robot:.3f}, offset={self.offset:.3f}')
                    self.get_logger().info('Stopping auto-drive and switching to LOCKED state.')

                    # LOCKED 들어감 + STOP 한 번 보내기
                    self.state = self.STATE_LOCKED
                    self.need_stop_once = True

                    # 재보정 초기화
                    self.last_lat = lat
                    self.last_lon = lon
                    self.recalc_heading_list = []
                    self.recalc_distance = 0.0
                    self.recalc_prev_east = 0.0
                    self.recalc_prev_north = 0.0
                    self.time_last_recalc = self.get_clock().now()
            return

        # 3) LOCKED 상태: 퍼블리시하면서 더 좋은 세그먼트 있는지 본다
        if self.state == self.STATE_LOCKED:
            dlat = math.radians(lat - self.last_lat)
            dlon = math.radians(lon - self.last_lon)
            east = 6378137.0 * dlon * math.cos(math.radians(self.last_lat))
            north = 6378137.0 * dlat
            self.recalc_distance = math.hypot(east, north)

            step_east = east - self.recalc_prev_east
            step_north = north - self.recalc_prev_north
            if math.hypot(step_east, step_north) > 0.001:
                step_heading = math.degrees(math.atan2(step_north, step_east))
                self.recalc_heading_list.append(step_heading)
            self.recalc_prev_east = east
            self.recalc_prev_north = north

            # 최소 5초 간격
            now = self.get_clock().now()
            elapsed = (now - self.time_last_recalc).nanoseconds * 1e-9

            if self.recalc_distance >= 5.0 and len(self.recalc_heading_list) >= 2 and elapsed >= 5.0:
                mean_r = sum(self.recalc_heading_list) / len(self.recalc_heading_list)
                var_r = sum((h - mean_r) ** 2 for h in self.recalc_heading_list) / len(self.recalc_heading_list)
                stddev_r = math.sqrt(var_r)

                if stddev_r <= 1.0:
                    # 더 좋은 세그먼트로 판단 → 새 offset
                    yaw_gps_new = math.atan2(north, east)
                    yaw_robot_now = self.yaw_robot or 0.0
                    new_offset = yaw_gps_new - yaw_robot_now
                    new_offset = math.atan2(math.sin(new_offset), math.cos(new_offset))
                    self.offset = new_offset

                    self.get_logger().info(
                        f'Recalibration: dist={self.recalc_distance:.2f} m, std={stddev_r:.2f} deg')
                    self.get_logger().info(
                        f'New GPS yaw={yaw_gps_new:.3f}, robot yaw={yaw_robot_now:.3f}, new offset={self.offset:.3f}')

                    # 다음 재보정 준비
                    self.last_lat = lat
                    self.last_lon = lon
                    self.recalc_heading_list = []
                    self.recalc_distance = 0.0
                    self.recalc_prev_east = 0.0
                    self.recalc_prev_north = 0.0
                    self.time_last_recalc = now


def main(args=None):
    rclpy.init(args=args)
    node = GPSHeadingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
