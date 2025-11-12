#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSV로 수집한 위/경도 리스트를 그래프로 만들고,
현재 위치(/fix 또는 수동 입력)에서 목적지(lat/lon)까지 Dijkstra 최단경로를 구한 뒤
Nav2 logged_waypoint_follower가 읽을 수 있는 YAML(poses/waypoints)을 생성한다.

- 기본 입력: --csv <lat,lon CSV>
  * 자동 컬럼탐지: lat, lon / latitude, longitude / x,y / gps_lat,gps_lon 등
- 시작점:
  * --current-lat/--current-lon 직접 지정, 또는
  * --use-fix 로 /fix (sensor_msgs/NavSatFix) 1건을 래치해서 사용
- 그래프 연결:
  * 인접한 점 + neighbor-radius-m 이내의 점을 연결 (부드러운 추종을 위해 3~6 m 권장)
- 출력:
  * default: /tmp/waypoints_nav2.yaml
  * YAML 안에 poses: [PoseStamped...] 와 waypoints: [PoseStamped...] 를 동시에 넣어 호환성 확보
"""

import math
import argparse
import sys
import csv
from collections import defaultdict
from typing import List, Tuple, Dict, Optional

# (--use-fix 용)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

R_EARTH = 6378137.0  # WGS84


# ---------------- Geodesy helpers ----------------
def haversine_m(lat1, lon1, lat2, lon2) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    a = math.sin(dlat/2)**2 + math.cos(lat1r)*math.cos(lat2r)*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R_EARTH * c


def wgs84_to_enu(lat_ref, lon_ref, lat, lon) -> Tuple[float, float]:
    dlat = math.radians(lat - lat_ref)
    dlon = math.radians(lon - lon_ref)
    east  = dlon * math.cos(math.radians(lat_ref)) * R_EARTH
    north = dlat * R_EARTH
    return east, north


def yaw_to_quat_z_w(yaw_rad: float) -> Tuple[float, float]:
    """2D yaw만 반영 (x=y=0, z,w만 사용)"""
    z = math.sin(yaw_rad * 0.5)
    w = math.cos(yaw_rad * 0.5)
    return z, w


# ---------------- CSV loader ----------------
def load_lat_lon_csv(path: str) -> List[Tuple[float, float]]:
    import pandas as pd
    import numpy as np

    df_raw = pd.read_csv(path)
    cols = [c.lower().strip() for c in df_raw.columns]
    lat_col = None; lon_col = None

    for cand in ["lat","latitude","gps_lat","y"]:
        if cand in cols:
            lat_col = df_raw.columns[cols.index(cand)]
            break
    for cand in ["lon","lng","longitude","gps_lon","x"]:
        if cand in cols:
            lon_col = df_raw.columns[cols.index(cand)]
            break

    if lat_col is None or lon_col is None:
        # fallback: 앞 2개 숫자 컬럼
        num_cols = [c for c in df_raw.columns if np.issubdtype(df_raw[c].dtype, np.number)]
        if len(num_cols) >= 2:
            lat_col, lon_col = num_cols[0], num_cols[1]
        else:
            raise ValueError(f"CSV에서 lat/lon 컬럼을 찾을 수 없음: {df_raw.columns.tolist()}")

    df = df_raw[[lat_col, lon_col]].rename(columns={lat_col:"lat", lon_col:"lon"}).copy()
    df = df.replace([np.inf, -np.inf], math.nan).dropna().reset_index(drop=True)
    pts = list(zip(df["lat"].tolist(), df["lon"].tolist()))
    if len(pts) < 2:
        raise ValueError("경로 포인트가 2개 미만입니다.")
    return pts


# ---------------- Graph build ----------------
def build_graph(points: List[Tuple[float,float]], neighbor_radius_m: float) -> Dict[int, List[Tuple[int, float]]]:
    """
    간단 그래프: (i,i+1) 연결 + neighbor_radius_m 이내의 j와도 연결
    간선 가중치 = 하버사인 거리
    """
    n = len(points)
    adj: Dict[int, List[Tuple[int, float]]] = defaultdict(list)

    # 항상 인접 연결
    for i in range(n-1):
        d = haversine_m(points[i][0], points[i][1], points[i+1][0], points[i+1][1])
        adj[i].append((i+1, d))
        adj[i+1].append((i, d))

    if neighbor_radius_m > 0.0:
        # 추가 연결 (너무 조밀해지지 않도록 i<j-1만 고려)
        for i in range(n):
            for j in range(i+2, n):  # i+1은 이미 연결되어 있음
                d = haversine_m(points[i][0], points[i][1], points[j][0], points[j][1])
                if d <= neighbor_radius_m:
                    adj[i].append((j, d))
                    adj[j].append((i, d))

    return adj


# ---------------- Dijkstra ----------------
def dijkstra(adj: Dict[int, List[Tuple[int, float]]], start: int, goal: int) -> List[int]:
    import heapq
    INF = float('inf')
    n = max(adj.keys()) + 1
    dist = [INF]*n
    prev = [-1]*n
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
    # path backtrack
    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


# ---------------- /fix latch ----------------
class FixLatch(Node):
    def __init__(self, topic_name: str):
        super().__init__('fix_latch')
        self._got = False
        self.lat = None
        self.lon = None
        self.create_subscription(NavSatFix, topic_name, self._cb, 10)

    def _cb(self, msg: NavSatFix):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.lat = float(msg.latitude)
            self.lon = float(msg.longitude)
            self._got = True


def latch_fix(topic: str, timeout_sec: float = 10.0) -> Optional[Tuple[float,float]]:
    rclpy.init(args=None)
    node = FixLatch(topic)
    from time import time
    t0 = time()
    while rclpy.ok() and (time() - t0) < timeout_sec and not node._got:
        rclpy.spin_once(node, timeout_sec=0.1)
    latlon = (node.lat, node.lon) if node._got else None
    node.destroy_node()
    rclpy.shutdown()
    return latlon


# ---------------- YAML writer ----------------
def write_nav2_yaml(path_xy: List[Tuple[float,float]], out_path: str, frame_id: str = "map") -> None:
    """
    PoseStamped 배열 형태로 'poses'와 'waypoints' 두 키에 동시에 써줌.
    각 포즈는 다음 점을 향한 yaw(마지막은 이전 점을 따라 yaw)를 사용.
    """
    import yaml

    def yaw_between(a: Tuple[float,float], b: Tuple[float,float]) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        return math.atan2(dy, dx)

    poses = []
    n = len(path_xy)
    for i, (x, y) in enumerate(path_xy):
        if n == 1:
            yaw = 0.0
        elif i < n-1:
            yaw = yaw_between(path_xy[i], path_xy[i+1])
        else:
            yaw = yaw_between(path_xy[i-1], path_xy[i])
        z_q, w_q = yaw_to_quat_z_w(yaw)
        ps = {
            "header": {"frame_id": frame_id},
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": float(z_q), "w": float(w_q)},
            }
        }
        poses.append(ps)

    out = {"poses": poses, "waypoints": poses}  # 호환성 위해 두 키 모두 기록
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)


# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser(
        description="Dijkstra shortest path on GPS waypoint graph -> Nav2 logged_waypoint_follower YAML"
    )
    ap.add_argument("--csv", required=True, help="CSV with headers: lat,lon (or latitude,longitude)")
    ap.add_argument("--dest-lat", required=True, type=float)
    ap.add_argument("--dest-lon", required=True, type=float)
    ap.add_argument("--current-lat", type=float, help="If omitted and --use-fix, latch /fix")
    ap.add_argument("--current-lon", type=float)
    ap.add_argument("--use-fix", action="store_true", help="Latch current position from /fix if --current-xxx not provided")
    ap.add_argument("--fix-topic", default="/fix")
    ap.add_argument("--neighbor-radius-m", type=float, default=5.0)
    ap.add_argument("--dedup-eps-m", type=float, default=0.5, help="Drop consecutive duplicates closer than this")
    ap.add_argument("--out", default="/tmp/waypoints_nav2.yaml")
    args = ap.parse_args()

    # 1) CSV 로드
    pts = load_lat_lon_csv(args.csv)

    # 2) 연속 중복 제거(선택)
    dedup_pts = [pts[0]]
    for p in pts[1:]:
        if haversine_m(dedup_pts[-1][0], dedup_pts[-1][1], p[0], p[1]) >= args.dedup_eps_m:
            dedup_pts.append(p)
    if dedup_pts[-1] != pts[-1]:
        dedup_pts.append(pts[-1])
    pts = dedup_pts

    # 3) 시작점 획득
    if args.current_lat is None or args.current_lon is None:
        if args.use_fix:
            print(f"[INFO] Latching current from {args.fix_topic} ...")
            cur = latch_fix(args.fix_topic, timeout_sec=10.0)
            if cur is None:
                print("[ERROR] Could not latch /fix; provide --current-lat/--current-lon", file=sys.stderr)
                sys.exit(2)
            cur_lat, cur_lon = cur
        else:
            print("[ERROR] Need --current-lat/--current-lon or --use-fix", file=sys.stderr)
            sys.exit(2)
    else:
        cur_lat, cur_lon = args.current_lat, args.current_lon

    # 4) 최근접 인덱스
    def nearest_idx(lat, lon) -> int:
        best_i = 0
        best_d = 1e18
        for i,(la,lo) in enumerate(pts):
            d = haversine_m(lat, lon, la, lo)
            if d < best_d:
                best_d = d; best_i = i
        return best_i

    start_i = nearest_idx(cur_lat, cur_lon)
    goal_i  = nearest_idx(args.dest_lat, args.dest_lon)
    print(f"[INFO] nearest start idx={start_i}, goal idx={goal_i} (start->goal forward path assumed)")

    # 5) 그래프 구성
    adj = build_graph(pts, neighbor_radius_m=args.neighbor_radius_m)

    # 6) 최단경로
    idx_path = dijkstra(adj, start_i, goal_i)
    if not idx_path:
        print("[ERROR] No path found in graph; try increasing --neighbor-radius-m", file=sys.stderr)
        sys.exit(3)

    # 7) ENU 좌표로 변환 (frame_id=map 가정) - datum은 첫 점 기준
# 7) ENU 좌표로 변환 (frame_id=map 가정)
# --use-fix로 현재 위치를 썼다면 그 좌표를 datum으로 사용, 아니면 CSV 첫 점
    if args.use_fix:
        lat0, lon0 = cur_lat, cur_lon
    else:
        lat0, lon0 = pts[0]

    path_xy = []
    for i in idx_path:
        x, y = wgs84_to_enu(lat0, lon0, pts[i][0], pts[i][1])  # x=East, y=North (ENU)
        path_xy.append((x, y))

    # 8) YAML 기록
    write_nav2_yaml(path_xy, args.out, frame_id="map")
    print(f"[OK] Wrote Nav2 YAML: {args.out}")
    print(f"[OK] Waypoints: {len(path_xy)}")


if __name__ == "__main__":
    main()


