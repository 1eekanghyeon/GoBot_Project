# ~/ros2_ws/src/cov_tools/live_cov_calib.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time, statistics, math

CALIB_SEC = 300.0  # 캘리브레이션 시간(정지)

class LiveCovCalib(Node):
    def __init__(self):
        super().__init__('live_cov_calib')
        self.xs, self.ys, self.gzs = [], [], []
        self.t0 = self.get_clock().now()
        self.sub_gps = self.create_subscription(Odometry, '/odometry/gps', self.gps_cb, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu_converted', self.imu_cb, 200)
        self.timer = self.create_timer(1.0, self.tick)

    def gps_cb(self, msg: Odometry):
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def imu_cb(self, msg: Imu):
        self.gzs.append(msg.angular_velocity.z)

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.get_logger().info(f'Collecting... {elapsed:.1f}/{CALIB_SEC:.0f}s  samples: gps={len(self.xs)}, imu={len(self.gzs)}')
        if elapsed >= CALIB_SEC:
            self.report()
            rclpy.shutdown()

    def robust_std(self, arr):
        if len(arr) < 30:
            return None
        # IQR 기반 간단한 이상치 제거
        qs = sorted(arr)
        q1 = qs[int(0.25*len(qs))]
        q3 = qs[int(0.75*len(qs))]
        iqr = q3 - q1
        low, high = q1 - 1.5*iqr, q3 + 1.5*iqr
        arr2 = [a for a in arr if low <= a <= high]
        if len(arr2) < 10: arr2 = arr
        return statistics.pstdev(arr2)

    def report(self):
        sx = self.robust_std(self.xs)
        sy = self.robust_std(self.ys)
        sgz = self.robust_std(self.gzs)

        print('\n==== Live Covariance Calibration Report ====')
        if sx is not None and sy is not None:
            vx = sx*sx; vy = sy*sy
            print(f'GPS position std (m):   σx={sx:.3f}, σy={sy:.3f}  → variances: {vx:.3f}, {vy:.3f}')
            # 게이트(마할라노비스) 러프 권장: 3σ 수준
            gate_m = 3.0
            thr = gate_m  # RL의 threshold는 "거리" 기준. 2D이면 2~5 사이에서 시작해 튜닝.
            print(f'Recommended odom1_pose_rejection_threshold ≈ {thr:.1f} (start with 2.0~5.0)')

            print('\nYAML (ekf_map, GPS 위치만 사용):')
            print(f'''    odom1: /odometry/gps
    odom1_config: [true, true, false,  false, false, false,  false, false, false,  false, false, false,  false, false, false]
    odom1_pose_rejection_threshold: {thr:.1f}
    odom1_queue_size: 5
    odom1_nodelay: true
''')
        else:
            print('Not enough /odometry/gps samples.')

        if sgz is not None:
            vz = sgz*sgz
            print(f'IMU gyro z std (rad/s): σgz={sgz:.5f}  → variance: {vz:.6f}')
            print('YAML (ekf_map에 vyaw만 쓰는 경우):')
            print('''    imu1: /imu_converted
    imu1_config: [false,false,false,  false,false,false,  false,false,false,  false,false,true,  false,false,false]
    imu1_remove_gravitational_acceleration: true
''')
        else:
            print('Not enough /imu_converted samples.')

def main():
    rclpy.init()
    node = LiveCovCalib()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
