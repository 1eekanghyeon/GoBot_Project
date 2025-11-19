#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RTK /fix + CSV + 3m 오프셋 제한 그래프 + Dijkstra + PID 주행 노드

- 이 노드는 네가 올린 route_constrained 코드의
  - haversine_m
  - load_lat_lon_csv
  - dedup_points
  - nearest_idx
  - edge_supported
  - build_graph
  - dijkstra
  를 그대로 사용해서 경로를 만들고,
- /fix(RTK)만 가지고 ENU 위치/heading을 추정해서
- /cmd_vel_out 으로 PID 주행을 한다.

필요한 것:
  - /fix (sensor_msgs/NavSatFix) : RTK GPS
  - CSV: 수집 궤적 (lat, lon)
  - dest_lat / dest_lon : 목표 위경도

안 쓰는 것:
  - EKF, navsat, Nav2, /odometry/filtered_* 전부 사용 안 함
"""

import math
from collections import deque
from typing import List, Tuple, Dict, Optional

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

R_EARTH = 6378137.0  # WGS84


# ---------------- 거리 함수 ----------------
def haversine_m(lat1, lon1, lat2, lon2) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R_EARTH * c


# ---------------- CSV 로드 ----------------
def load_lat_lon_csv(path: str) -> List[Tuple[float, float]]:
    df = pd.read_csv(path)
    cols = [c.lower().strip() for c in df.columns]

    lat_col = None
    lon_col = None
    for cand in ["lat", "latitude", "gps_lat", "y"]:
        if cand in cols:
            lat_col = df.columns[cols.index(cand)]
            break
    for cand in ["lon", "lng", "longitude", "gps_lon", "x"]:
        if cand in cols:
            lon_col = df.columns[cols.index(cand)]
            break

    if lat_col is None or lon_col is None:
        num_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
        if len(num_cols) < 2:
            raise ValueError("CSV에서 lat/lon 컬럼 못 찾음")
        lat_col, lon_col = num_cols[0], num_cols[1]

    df = df[[lat_col, lon_col]].copy()
    df.columns = ["lat", "lon"]
    df = df.replace([np.inf, -np.inf], np.nan).dropna().reset_index(drop=True)
    return list(zip(df["lat"].tolist(), df["lon"].tolist()))


# ---------------- 다운샘플 (노드용) ----------------
def dedup_points(pts: List[Tuple[float, float]], eps_m: float = 0.5) -> List[Tuple[float, float]]:
    out = [pts[0]]
    for p in pts[1:]:
        if haversine_m(out[-1][0], out[-1][1], p[0], p[1]) >= eps_m:
            out.append(p)
    return out


# ---------------- 최근접 인덱스 ----------------
def nearest_idx(pts: List[Tuple[float, float]], lat: float, lon: float) -> int:
    best_i = 0
    best_d = 1e18
    for i, (la, lo) in enumerate(pts):
        d = haversine_m(lat, lon, la, lo)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


# ---------------- 엣지 유효성 체크 ----------------
def edge_supported(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    raw_pts: List[Tuple[float, float]],
    step_m: float = 1.0,
    max_off_m: float = 3.0,
) -> bool:
    total_d = haversine_m(p1[0], p1[1], p2[0], p2[1])
    if total_d <= step_m:
        return True

    n_steps = max(2, int(total_d / step_m))
    for k in range(1, n_steps):  # 끝점은 어차피 raw point에 붙어있음
        t = k / n_steps
        lat = p1[0] + (p2[0] - p1[0]) * t
        lon = p1[1] + (p2[1] - p1[1]) * t

        best = 1e18
        for (rla, rlo) in raw_pts:
            d = haversine_m(lat, lon, rla, rlo)
            if d < best:
                best = d
                if best <= max_off_m:  # 3m 안에 하나라도 있으면 OK
                    break

        if best > max_off_m:
            return False   # 이 샘플 지점이 수집된 경로에서 3m 넘게 떨어짐 -> 엣지 폐기

    return True


# ---------------- 그래프 생성 ----------------
def build_graph(
    nodes: List[Tuple[float, float]],
    raw_pts: List[Tuple[float, float]],
    neighbor_radius_m: float = 5.0,
    step_m: float = 1.0,
    max_off_m: float = 3.0,
):
    n = len(nodes)
    adj: List[List[Tuple[int, float]]] = [[] for _ in range(n)]

    # 1) 순차 이웃 (i <-> i+1)
    for i in range(n - 1):
        p1, p2 = nodes[i], nodes[i + 1]
        if edge_supported(p1, p2, raw_pts, step_m, max_off_m):
            d = haversine_m(p1[0], p1[1], p2[0], p2[1])
            adj[i].append((i + 1, d))
            adj[i + 1].append((i, d))

    # 2) 근접 노드 추가 연결
    for i in range(n):
        for j in range(i + 2, n):
            d = haversine_m(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])
            if d <= neighbor_radius_m:
                if edge_supported(nodes[i], nodes[j], raw_pts, step_m, max_off_m):
                    adj[i].append((j, d))
                    adj[j].append((i, d))

    return adj


# ---------------- 다익스트라 ----------------
def dijkstra(adj, start, goal):
    import heapq

    n = len(adj)
    INF = float("inf")
    dist = [INF] * n
    prev = [-1] * n
    dist[start] = 0.0
    pq = [(0.0, start)]

    while pq:
        cd, u = heapq.heappop(pq)
        if cd > dist[u]:
            continue
        if u == goal:
            break
        for v, w in adj[u]:
            nd = cd + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if dist[goal] == INF:
        return []

    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


# ---------------- 좌표계/각도 유틸 ----------------
def wgs84_to_enu(lat_ref: float, lon_ref: float, lat: float, lon: float) -> Tuple[float, float]:
    dlat = math.radians(lat - lat_ref)
    dlon = math.radians(lon - lon_ref)
    east = dlon * math.cos(math.radians(lat_ref)) * R_EARTH
    north = dlat * R_EARTH
    return east, north


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def circular_mean(angles: List[float]) -> float:
    if not angles:
        return 0.0
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


# ---------------- PID ----------------
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, i_min: float = -1.0, i_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_min = i_min
        self.i_max = i_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time: Optional[float] = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def step(self, error: float, now_sec: float) -> float:
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = max(1e-4, now_sec - self.prev_time)

        self.integral += error * dt
        self.integral = max(self.i_min, min(self.i_max, self.integral))

        derivative = 0.0
        if dt > 0.0:
            derivative = (error - self.prev_error) / dt

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.prev_time = now_sec
        return out


# ---------------- 메인 노드 ----------------
class GPSRTKPIDNav(Node):
    """
    /fix 기반 RTK GPS 네비게이션 (EKF/navsat/Nav2 없음)

    상태:
      - WAIT_FIX: 첫 fix + 경로 계산
      - INIT_STRAIGHT: 초기 직진 (옵션)
      - FOLLOW_PATH: 웨이포인트 PID 추종
      - DONE: 목표 도달
      - ABORT: 실패
    """

    STATE_WAIT_FIX = 0
    STATE_INIT_STRAIGHT = 1
    STATE_FOLLOW_PATH = 2
    STATE_DONE = 3
    STATE_ABORT = 4

    def __init__(self):
        super().__init__('gps_rtk_pid_nav')

        # ---- 파라미터 선언 ----
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_out')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('dest_lat', 0.0)
        self.declare_parameter('dest_lon', 0.0)

        self.declare_parameter('use_fix_as_start', True)
        self.declare_parameter('start_lat', 0.0)
        self.declare_parameter('start_lon', 0.0)

        self.declare_parameter('neighbor_radius_m', 5.0)
        self.declare_parameter('max_off_m', 3.0)
        self.declare_parameter('dedup_eps_m', 0.5)
        self.declare_parameter('edge_step_m', 1.0)
        self.declare_parameter('nav_pause_topic', '/nav_pause')

        self.declare_parameter('min_fix_status', 0)  # NavSatStatus.status >= 0
        self.declare_parameter('max_fix_age_sec', 1.0)

        # 초기 직진
        self.declare_parameter('init_straight_distance', 3.0)
        self.declare_parameter('init_straight_speed', 0.3)
        self.declare_parameter('init_straight_timeout', 20.0)

        # 제어 주기
        self.declare_parameter('control_rate', 20.0)

        # PID, 속도
        self.declare_parameter('kp_ang', 1.8)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.3)

        self.declare_parameter('max_ang_vel', 1.5)
        self.declare_parameter('max_lin_vel', 0.6)
        self.declare_parameter('min_lin_vel', 0.0)

        self.declare_parameter('slowdown_angle_deg', 60.0)

        self.declare_parameter('waypoint_xy_tolerance', 0.5)
        self.declare_parameter('final_waypoint_xy_tolerance', 0.8)
        self.declare_parameter('use_nearest_lookahead', True)
        self.declare_parameter('nearest_lookahead_count', 5)

        # heading 추정용
        self.declare_parameter('heading_window_size', 20)
        self.declare_parameter('heading_min_step_m', 0.02)

        # ---- 파라미터 값 읽기 ----
        self.fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.dest_lat = float(self.get_parameter('dest_lat').value)
        self.dest_lon = float(self.get_parameter('dest_lon').value)

        self.use_fix_as_start = bool(self.get_parameter('use_fix_as_start').value)
        self.start_lat_param = float(self.get_parameter('start_lat').value)
        self.start_lon_param = float(self.get_parameter('start_lon').value)

        self.neighbor_radius_m = float(self.get_parameter('neighbor_radius_m').value)
        self.max_off_m = float(self.get_parameter('max_off_m').value)
        self.dedup_eps_m = float(self.get_parameter('dedup_eps_m').value)
        self.edge_step_m = float(self.get_parameter('edge_step_m').value)

        self.min_fix_status = int(self.get_parameter('min_fix_status').value)
        self.max_fix_age_sec = float(self.get_parameter('max_fix_age_sec').value)

        self.init_straight_distance = float(self.get_parameter('init_straight_distance').value)
        self.init_straight_speed = float(self.get_parameter('init_straight_speed').value)
        self.init_straight_timeout = float(self.get_parameter('init_straight_timeout').value)

        self.control_rate = float(self.get_parameter('control_rate').value)

        kp_ang = float(self.get_parameter('kp_ang').value)
        ki_ang = float(self.get_parameter('ki_ang').value)
        kd_ang = float(self.get_parameter('kd_ang').value)

        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)
        self.max_lin_vel = float(self.get_parameter('max_lin_vel').value)
        self.min_lin_vel = float(self.get_parameter('min_lin_vel').value)

        self.slowdown_angle_deg = float(self.get_parameter('slowdown_angle_deg').value)

        self.waypoint_xy_tolerance = float(self.get_parameter('waypoint_xy_tolerance').value)
        self.final_waypoint_xy_tolerance = float(self.get_parameter('final_waypoint_xy_tolerance').value)
        self.use_nearest_lookahead = bool(self.get_parameter('use_nearest_lookahead').value)
        self.nearest_lookahead_count = int(self.get_parameter('nearest_lookahead_count').value)

        self.heading_window_size = int(self.get_parameter('heading_window_size').value)
        self.heading_min_step_m = float(self.get_parameter('heading_min_step_m').value)
        self.nav_pause_topic = self.get_parameter('nav_pause_topic').get_parameter_value().string_value
        # ----- low-pass 필터 계수 (0~1) -----
        self.pos_lp_alpha = 0.3   # 위치(x,y) 필터 강도
        self.yaw_lp_alpha = 0.3   # heading(yaw) 필터 강도
        # ---- 상태 ----
        self.state = self.STATE_WAIT_FIX
        self.paused: bool = False
        # CSV 포인트
        self.raw_pts: List[Tuple[float, float]] = []
        self.nodes: List[Tuple[float, float]] = []      # dedup 결과 (그래프 노드)
        self.path_xy: List[Tuple[float, float]] = []    # 최종 경로 (ENU x,y)

        # ENU 기준점 (첫 유효 fix 기준)
        self.lat0: Optional[float] = None
        self.lon0: Optional[float] = None

        # 현재 /fix
        self.current_lat: Optional[float] = None
        self.current_lon: Optional[float] = None
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.last_fix_time: Optional[float] = None

        # heading 추정용
        self.prev_x_for_heading: Optional[float] = None
        self.prev_y_for_heading: Optional[float] = None
        self.heading_samples: deque = deque(maxlen=self.heading_window_size)
        self.yaw_est: Optional[float] = None

        # INIT_STRAIGHT
        self.init_x: Optional[float] = None
        self.init_y: Optional[float] = None
        self.init_start_time: Optional[float] = None

        # path follow
        self.current_wp_idx: int = 0
        self.done_stop_sent: bool = False

        # PID
        self.pid_ang = PIDController(kp_ang, ki_ang, kd_ang, i_min=-1.0, i_max=1.0)

        # ---- CSV 로드 ----
        if not self.csv_path:
            self.get_logger().error('csv_path 파라미터가 비어있습니다. ABORT.')
            self.state = self.STATE_ABORT
        else:
            try:
                pts = load_lat_lon_csv(self.csv_path)
            except Exception as e:
                self.get_logger().error(f'CSV 로드 실패: {self.csv_path} -> {e}')
                self.state = self.STATE_ABORT
                pts = []
            else:
                self.raw_pts = pts
                self.nodes = dedup_points(self.raw_pts, eps_m=self.dedup_eps_m)
                self.get_logger().info(
                    f'CSV 로드 완료: raw_pts={len(self.raw_pts)}, nodes(dedup)={len(self.nodes)}'
                )

        if self.dest_lat == 0.0 and self.dest_lon == 0.0:
            self.get_logger().warn('dest_lat/dest_lon 이 0.0입니다. 실제 목적지 좌표로 바꿔야 함.')

        # ---- ROS I/O ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(NavSatFix, self.fix_topic, self._fix_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Bool, self.nav_pause_topic, self._pause_cb, 10)
        # ---- 타이머 ----
        if self.control_rate <= 0.0:
            self.control_rate = 20.0
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info('gps_rtk_pid_nav node started.')
        self.get_logger().info(f'fix_topic={self.fix_topic}, cmd_vel_topic={self.cmd_vel_topic}')
        self.get_logger().info(f'csv_path={self.csv_path}, dest_lat={self.dest_lat}, dest_lon={self.dest_lon}')

    # ---------- /fix 콜백 ----------
    def _fix_cb(self, msg: NavSatFix):
        if msg.status.status < self.min_fix_status:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.last_fix_time = now_sec

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        if not math.isfinite(lat) or not math.isfinite(lon):
            return

        if self.lat0 is None or self.lon0 is None:
            self.lat0 = lat
            self.lon0 = lon
            self.get_logger().info(
                f'ENU origin set to first fix: lat0={self.lat0:.8f}, lon0={self.lon0:.8f}'
            )

        x_raw, y_raw = wgs84_to_enu(self.lat0, self.lon0, lat, lon)
        # ----- 위치(x, y) low-pass 필터 -----
        #   current_x/current_y가 없으면 첫 값은 그냥 raw 사용
        if self.current_x is None or self.current_y is None:
            x = x_raw
            y = y_raw
        else:
            a = self.pos_lp_alpha
            x = (1.0 - a) * self.current_x + a * x_raw
            y = (1.0 - a) * self.current_y + a * y_raw

        # heading 추정
        if self.prev_x_for_heading is not None and self.prev_y_for_heading is not None:
            dx = x - self.prev_x_for_heading
            dy = y - self.prev_y_for_heading
            step_dist = math.hypot(dx, dy)
            if step_dist >= self.heading_min_step_m:
                step_yaw = math.atan2(dy, dx)
                self.heading_samples.append(step_yaw)
                # 최근 몇 개 step_yaw의 circular mean (원래 있던 평균)
                raw_yaw = circular_mean(list(self.heading_samples))

                # 여기에 yaw low-pass 한 번 더
                if self.yaw_est is None:
                    self.yaw_est = raw_yaw
                else:
                    b = self.yaw_lp_alpha
                    self.yaw_est = normalize_angle(
                        (1.0 - b) * self.yaw_est + b * raw_yaw
                    )
        self.prev_x_for_heading = x
        self.prev_y_for_heading = y

        self.current_lat = lat
        self.current_lon = lon
        self.current_x = x
        self.current_y = y
    # ---------- /nav_pause 콜백 ----------
    def _pause_cb(self, msg: Bool):
        new = bool(msg.data)
        if new != self.paused:
            self.paused = new
            state_str = "PAUSE" if new else "RESUME"
            self.get_logger().info(f'/nav_pause -> {state_str}')
        else:
            self.paused = new

    # ---------- 메인 타이머 ----------
    def _timer_cb(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # fix 오래됐으면 STOP
        if self.last_fix_time is not None and self.max_fix_age_sec > 0.0:
            if now_sec - self.last_fix_time > self.max_fix_age_sec:
                self.get_logger().warn_once('FIX too old; STOP.')
                self._publish_stop()
                return

        if self.paused:
            self._publish_stop()
            return

        if self.state == self.STATE_ABORT:
            if not self.done_stop_sent:
                self.get_logger().error('STATE_ABORT -> STOP.')
                self._publish_stop()
                self.done_stop_sent = True
            return

        if self.state == self.STATE_DONE:
            if not self.done_stop_sent:
                self.get_logger().info('STATE_DONE -> final STOP.')
                self._publish_stop()
                self.done_stop_sent = True
            return

        if self.state == self.STATE_WAIT_FIX:
            self._step_wait_fix(now_sec)
        elif self.state == self.STATE_INIT_STRAIGHT:
            self._step_init_straight(now_sec)
        elif self.state == self.STATE_FOLLOW_PATH:
            self._step_follow_path(now_sec)

    # ---------- WAIT_FIX 단계 ----------
    def _step_wait_fix(self, now_sec: float):
        if self.current_lat is None or self.current_lon is None:
            self._publish_stop()
            return

        if not self.raw_pts or not self.nodes:
            self.get_logger().error('raw_pts/nodes 비어있음. ABORT.')
            self.state = self.STATE_ABORT
            return

        if self.dest_lat == 0.0 and self.dest_lon == 0.0:
            self.get_logger().error('dest_lat/dest_lon 이 0.0. ABORT.')
            self.state = self.STATE_ABORT
            return

        # 시작점 결정
        if self.use_fix_as_start:
            start_lat = self.current_lat
            start_lon = self.current_lon
        else:
            start_lat = self.start_lat_param
            start_lon = self.start_lon_param

        # 시작/목표 인덱스 (nodes 기반)
        start_idx = nearest_idx(self.nodes, start_lat, start_lon)
        goal_idx = nearest_idx(self.nodes, self.dest_lat, self.dest_lon)

        self.get_logger().info(
            f'[PATH] start_idx={start_idx}, goal_idx={goal_idx}, '
            f'start_lat={start_lat:.8f}, start_lon={start_lon:.8f}'
        )

        # 그래프 생성 (3m 규칙 적용)
        adj = build_graph(
            self.nodes,
            self.raw_pts,
            neighbor_radius_m=self.neighbor_radius_m,
            step_m=self.edge_step_m,
            max_off_m=self.max_off_m,
        )

        idx_path = dijkstra(adj, start_idx, goal_idx)
        if not idx_path:
            self.get_logger().error('[PATH] 조건 만족하는 경로 없음 (3m 규칙 때문에 막혔을 수 있음). ABORT.')
            self.state = self.STATE_ABORT
            return

        # ENU 경로로 변환 (lat0/lon0 기준)
        if self.lat0 is None or self.lon0 is None:
            self.get_logger().error('lat0/lon0 없음 -> ENU 변환 불가. ABORT.')
            self.state = self.STATE_ABORT
            return

        self.path_xy = []
        for i in idx_path:
            la, lo = self.nodes[i]
            x, y = wgs84_to_enu(self.lat0, self.lon0, la, lo)
            self.path_xy.append((x, y))

        self.get_logger().info(
            f'[PATH] 최종 ENU 경로 길이: {len(self.path_xy)} (raw_pts={len(self.raw_pts)}, nodes={len(self.nodes)})'
        )

        self.current_wp_idx = 0
        self.pid_ang.reset()
        self.done_stop_sent = False
        self.init_x = None
        self.init_y = None
        self.init_start_time = None

        if self.init_straight_distance > 0.0:
            self.state = self.STATE_INIT_STRAIGHT
            self.get_logger().info(
                f'Switch to INIT_STRAIGHT: dist={self.init_straight_distance:.2f} m, v={self.init_straight_speed:.2f}'
            )
        else:
            self.state = self.STATE_FOLLOW_PATH
            self.get_logger().info('Switch to FOLLOW_PATH.')
            self._publish_stop()

    # ---------- INIT_STRAIGHT 단계 ----------
    def _step_init_straight(self, now_sec: float):
        if self.current_x is None or self.current_y is None:
            self._publish_stop()
            return

        if self.init_x is None or self.init_y is None:
            self.init_x = self.current_x
            self.init_y = self.current_y
            self.init_start_time = now_sec
            self.get_logger().info(
                f'INIT_STRAIGHT 시작: ({self.init_x:.2f}, {self.init_y:.2f}), '
                f'dist={self.init_straight_distance:.2f} m'
            )

        dx = self.current_x - self.init_x
        dy = self.current_y - self.init_y
        dist = math.hypot(dx, dy)
        elapsed = now_sec - (self.init_start_time or now_sec)

        if dist >= self.init_straight_distance:
            self.get_logger().info(
                f'INIT_STRAIGHT 완료: dist={dist:.2f}, elapsed={elapsed:.1f}s -> FOLLOW_PATH'
            )
            self.state = self.STATE_FOLLOW_PATH
            self.pid_ang.reset()
            self._publish_stop()
            return

        if elapsed > self.init_straight_timeout:
            self.get_logger().warn(
                f'INIT_STRAIGHT timeout: dist={dist:.2f} in {elapsed:.1f}s -> FOLLOW_PATH'
            )
            self.state = self.STATE_FOLLOW_PATH
            self.pid_ang.reset()
            self._publish_stop()
            return

        cmd = Twist()
        cmd.linear.x = self.init_straight_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ---------- FOLLOW_PATH 단계 ----------
    def _step_follow_path(self, now_sec: float):
        if not self.path_xy or self.current_x is None or self.current_y is None:
            self._publish_stop()
            return

        n = len(self.path_xy)
        if self.current_wp_idx >= n:
            self.get_logger().info('모든 웨이포인트 소비 -> DONE.')
            self.state = self.STATE_DONE
            return

        if self.yaw_est is None:
            self.get_logger().warn_once('yaw_est 없음 -> 임시 직진.')
            cmd = Twist()
            cmd.linear.x = min(0.3, self.max_lin_vel)
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        idx = self.current_wp_idx

        # lookahead 기반 index 업데이트
        if self.use_nearest_lookahead:
            max_idx = min(n - 1, idx + self.nearest_lookahead_count)
            best_idx = idx
            best_dist = float('inf')
            for i in range(idx, max_idx + 1):
                wx, wy = self.path_xy[i]
                d = math.hypot(wx - self.current_x, wy - self.current_y)
                if d < best_dist:
                    best_dist = d
                    best_idx = i
            idx = best_idx
            self.current_wp_idx = idx

        wx, wy = self.path_xy[idx]
        dx = wx - self.current_x
        dy = wy - self.current_y
        dist = math.hypot(dx, dy)

        is_last = (idx == n - 1)
        tol = self.final_waypoint_xy_tolerance if is_last else self.waypoint_xy_tolerance

        if dist <= tol:
            self.get_logger().info(
                f'WP[{idx}/{n-1}] 도달: dist={dist:.2f} <= tol={tol:.2f}'
            )
            self.current_wp_idx += 1
            self._publish_stop()
            if self.current_wp_idx >= n:
                self.get_logger().info('모든 웨이포인트 도달 -> DONE.')
                self.state = self.STATE_DONE
            return

        target_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(target_yaw - self.yaw_est)

        ang_raw = self.pid_ang.step(heading_error, now_sec)
        ang_cmd = max(-self.max_ang_vel, min(self.max_ang_vel, ang_raw))

        # heading 오차에 따른 속도 스케일
        heading_deg = abs(math.degrees(heading_error))
        slow_angle = max(1e-3, self.slowdown_angle_deg)

        if heading_deg >= slow_angle * 2.0:
            angle_scale = 0.0
        else:
            if heading_deg <= slow_angle:
                angle_scale = 1.0
            else:
                t = (heading_deg - slow_angle) / slow_angle
                angle_scale = max(0.0, 1.0 - t)

        # 거리 기반 스케일
        if dist < 1.0:
            dist_scale = dist
        else:
            dist_scale = 1.0

        lin_scale = angle_scale * dist_scale
        lin_cmd = self.max_lin_vel * lin_scale
        if lin_cmd < self.min_lin_vel:
            lin_cmd = 0.0

        cmd = Twist()
        cmd.linear.x = lin_cmd
        cmd.angular.z = ang_cmd
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'FOLLOW: WP[{idx}/{n-1}] dist={dist:.2f}, '
            f'heading_err={math.degrees(heading_error):.1f}deg, '
            f'lin={lin_cmd:.2f}, ang={ang_cmd:.2f}'
        )

    # ---------- STOP ----------
    def _publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GPSRTKPIDNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
