#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import pynmea2


def f(x):
    try:
        return float(x)
    except Exception:
        return 0.0


class TcpNmeaToNavSatFix(Node):
    def __init__(self):
        super().__init__('tcp_nmea_to_navsatfix')

        # ── 파라미터 ───────────────────────────────────────────────────────────────
        self.declare_parameter('server_host', '192.168.0.100')  # TCP 서버 IP (Jetson 등)
        self.declare_parameter('server_port', 5000)             # TCP 서버 포트
        self.declare_parameter('frame_id', 'gps_link')          # NavSatFix header.frame_id
        self.declare_parameter('topic_name', '/fix')            # 퍼블리시 토픽
        self.declare_parameter('strict_rtk_only', True)         # RTK fixed(qual=4)만 유효로 볼지

        self.host = self.get_parameter('server_host').value
        self.port = int(self.get_parameter('server_port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic_name').value
        self.strict_rtk_only = bool(self.get_parameter('strict_rtk_only').value)

        # ── QoS (GNSS는 종종 BEST_EFFORT) ─────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(NavSatFix, self.topic, qos)

        self.count = 0
        self.get_logger().info(f'Connecting to {self.host}:{self.port}, publishing {self.topic}')
        self.task = asyncio.create_task(self._connect_loop())

    async def _connect_loop(self):
        while rclpy.ok():
            try:
                reader, _ = await asyncio.open_connection(self.host, self.port)
                self.get_logger().info('TCP connected.')
                await self._read(reader)
            except Exception as e:
                self.get_logger().warning(f'connect error: {e}; retry in 1s')
                await asyncio.sleep(1.0)

    async def _read(self, reader: asyncio.StreamReader):
        while rclpy.ok():
            line = await reader.readline()
            if not line:
                await asyncio.sleep(0.01)
                continue

            s = line.decode('utf-8', 'ignore').strip()
            if not s.startswith('$'):
                continue

            try:
                msg = pynmea2.parse(s)
            except Exception:
                continue

            # GGA 계열만 사용 (GPGGA/GNGGA 모두 대응)
            if type(msg).__name__.endswith('GGA'):
                self._publish_gga(msg)

    def _publish_gga(self, gga):
        # 필수 필드 체크
        if not getattr(gga, 'latitude', None) or not getattr(gga, 'longitude', None):
            return

        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.latitude = f(getattr(gga, 'latitude', 0.0))
        m.longitude = f(getattr(gga, 'longitude', 0.0))
        m.altitude = f(getattr(gga, 'altitude', 0.0))

        # NMEA GGA gps_qual:
        # 0=invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float, ...
        try:
            q = int(getattr(gga, 'gps_qual', 0) or 0)
        except Exception:
            q = 0

        if self.strict_rtk_only:
            # ★ 엄격 모드: RTK Fixed(qual=4)만 유효(=2)로 내보냄, 나머지는 전부 NO_FIX(=-1)
            if q == 4:
                m.status.status = NavSatStatus.STATUS_GBAS_FIX  # = 2
                m.status.service = (
                    NavSatStatus.SERVICE_GPS
                    | NavSatStatus.SERVICE_GLONASS
                    | NavSatStatus.SERVICE_GALILEO
                )
            else:
                m.status.status = NavSatStatus.STATUS_NO_FIX     # = -1
                m.status.service = 0
        else:
            if q in (4, 5):
                m.status.status = NavSatStatus.STATUS_GBAS_FIX   # = 2 (여기서 4를 그대로 쓰면 안 됨)
                m.status.service = (
                    NavSatStatus.SERVICE_GPS
                    | NavSatStatus.SERVICE_GLONASS
                    | NavSatStatus.SERVICE_GALILEO
                )
            else:
                return

        try:
            hdop = float(getattr(gga, 'horizontal_dil', 1.0) or 1.0)
        except Exception:
            hdop = 1.0

        # GPS 품질에 따라 "기본" 1σ(m) 설정 (대충 합리적인 값으로)
        if q == 4:          # RTK Fixed
            base_sigma_xy = 0.02   # 2cm
        elif q == 5:        # RTK Float
            base_sigma_xy = 0.25
        elif q == 2:        # DGPS
            base_sigma_xy = 0.5
        elif q == 1:        # 일반 GPS
            base_sigma_xy = 3.0
        else:               # invalid or unknown
            base_sigma_xy = 1000.0  # 거의 안 믿음

        sigma_xy = base_sigma_xy * hdop      # [m]
        var_xy = sigma_xy ** 2               # [m^2]
        sigma_z = 2.0 * sigma_xy             # 수직은 대충 2배 정도로
        var_z = sigma_z ** 2

        m.position_covariance = [
            var_xy, 0.0,   0.0,
            0.0,   var_xy, 0.0,
            0.0,   0.0,   var_z,
        ]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub.publish(m)
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(
                f'published {self.count} NavSatFix (qual={q}, hdop={hdop}, σxy≈{sigma_xy:.3f} m)'
            )

async def main_async():
    rclpy.init()
    node = TcpNmeaToNavSatFix()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
