#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSV(위도/경도)로 만든 그래프 위에서
현재(lat,lon) → 목적지(lat,lon) 최단경로(다익스트라)를 찾고
folium(Leaflet) HTML 지도로 시각화.

- 플래너(gps_path_planner.py)와 동일 로직:
  * 하버사인 거리
  * (i,i+1) 인접 연결 + neighbor-radius 내의 추가 연결
  * 연속 중복 제거(dedup-eps-m)
- ROS 불필요. /fix도 쓰지 않음(수동 lat/lon 입력).
"""

import math
import argparse
from collections import defaultdict
from typing import List, Tuple, Dict

import pandas as pd
import numpy as np
import folium

R_EARTH = 6378137.0  # WGS84 반경(m)

# ---------- 지오데시 헬퍼 ----------
def haversine_m(lat1, lon1, lat2, lon2) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    a = math.sin(dlat/2)**2 + math.cos(lat1r)*math.cos(lat2r)*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R_EARTH * c

# ---------- CSV 로더(플래너와 동일 자동 컬럼탐지) ----------
def load_lat_lon_csv(path: str) -> List[Tuple[float, float]]:
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

# ---------- 그래프 생성(인접 + 반경 연결) ----------
def build_graph(points: List[Tuple[float,float]], neighbor_radius_m: float) -> Dict[int, List[Tuple[int, float]]]:
    n = len(points)
    adj: Dict[int, List[Tuple[int, float]]] = defaultdict(list)

    # 항상 인접(i,i+1) 연결
    for i in range(n-1):
        d = haversine_m(points[i][0], points[i][1], points[i+1][0], points[i+1][1])
        adj[i].append((i+1, d))
        adj[i+1].append((i, d))

    if neighbor_radius_m > 0.0:
        for i in range(n):
            for j in range(i+2, n):  # i+1은 이미 연결됨
                d = haversine_m(points[i][0], points[i][1], points[j][0], points[j][1])
                if d <= neighbor_radius_m:
                    adj[i].append((j, d))
                    adj[j].append((i, d))
    return adj

# ---------- 다익스트라 ----------
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
    # backtrack
    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path

# ---------- 최근접 인덱스 ----------
def nearest_idx(pts: List[Tuple[float,float]], lat: float, lon: float) -> int:
    best_i = 0
    best_d = 1e18
    for i,(la,lo) in enumerate(pts):
        d = haversine_m(lat, lon, la, lo)
        if d < best_d:
            best_d = d; best_i = i
    return best_i

# ---------- 총거리 ----------
def path_length_m(pts: List[Tuple[float,float]]) -> float:
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(len(pts)-1):
        total += haversine_m(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
    return total

# ---------- 메인 ----------
def main():
    ap = argparse.ArgumentParser(description="GPS 경로 최단경로 시각화(HTML 지도)")
    ap.add_argument("--csv", required=True, help="CSV with lat,lon")
    ap.add_argument("--current-lat", required=True, type=float)
    ap.add_argument("--current-lon", required=True, type=float)
    ap.add_argument("--dest-lat", required=True, type=float)
    ap.add_argument("--dest-lon", required=True, type=float)
    ap.add_argument("--neighbor-radius-m", type=float, default=5.0)
    ap.add_argument("--dedup-eps-m", type=float, default=0.5)  # 연속 중복 제거 임계
    ap.add_argument("--out", default="/tmp/route.html")
    args = ap.parse_args()

    # 1) CSV 로드
    pts = load_lat_lon_csv(args.csv)

    # 2) 연속 중복 제거(앞점과 0.5m 미만이면 생략 등)
    dedup_pts = [pts[0]]
    for p in pts[1:]:
        if haversine_m(dedup_pts[-1][0], dedup_pts[-1][1], p[0], p[1]) >= args.dedup_eps_m:
            dedup_pts.append(p)
    if dedup_pts[-1] != pts[-1]:
        dedup_pts.append(pts[-1])
    pts = dedup_pts

    # 3) 시작/목적 노드 선택(최근접)
    start_i = nearest_idx(pts, args.current_lat, args.current_lon)
    goal_i  = nearest_idx(pts, args.dest_lat, args.dest_lon)

    # 4) 그래프 생성 & 다익스트라
    adj = build_graph(pts, neighbor_radius_m=args.neighbor_radius_m)
    idx_path = dijkstra(adj, start_i, goal_i)
    if not idx_path:
        raise SystemExit("No path found. --neighbor-radius-m 를 키워보세요.")

    path_pts = [pts[i] for i in idx_path]
    total_m = path_length_m(path_pts)

    # 5) folium 지도 시각화
    # 중심: (현재위치, 목적지, 경로 평균)
    center_lat = (args.current_lat + args.dest_lat + sum(p[0] for p in path_pts)/len(path_pts)) / 3.0
    center_lon = (args.current_lon + args.dest_lon + sum(p[1] for p in path_pts)/len(path_pts)) / 3.0
    m = folium.Map(location=[center_lat, center_lon], zoom_start=18, control_scale=True)

    # (a) 수집 전체 라인(얇은 회색)
    folium.PolyLine(pts, color="#999999", weight=2, opacity=0.6, tooltip="Recorded CSV path").add_to(m)

    # (b) 최단 경로(굵은 파란색)
    folium.PolyLine(path_pts, color="#0066ff", weight=5, opacity=0.9,
                    tooltip=f"Shortest path ({len(path_pts)} pts, {total_m:.1f} m)").add_to(m)

    # (c) 현재 위치(초록) / 목적지(빨강) 마커
    folium.Marker([args.current_lat, args.current_lon],
                  icon=folium.Icon(color="green", icon="play"),
                  tooltip="Current (입력값)").add_to(m)
    folium.Marker([args.dest_lat, args.dest_lon],
                  icon=folium.Icon(color="red", icon="flag"),
                  tooltip="Destination (입력값)").add_to(m)

    # (d) 사용된 시작/목적 노드(노랑/보라)
    folium.CircleMarker(pts[start_i], radius=5, color="#ffaa00", fill=True, fill_opacity=1.0,
                        tooltip=f"Start node idx={start_i}").add_to(m)
    folium.CircleMarker(pts[goal_i], radius=5, color="#9933ff", fill=True, fill_opacity=1.0,
                        tooltip=f"Goal node idx={goal_i}").add_to(m)

    # 6) 저장
    m.save(args.out)
    print(f"[OK] Saved map: {args.out}")
    print(f"[INFO] path points: {len(path_pts)}, length: {total_m:.2f} m")
    print(f"[INFO] start idx: {start_i}, goal idx: {goal_i}")

if __name__ == "__main__":
    main()
