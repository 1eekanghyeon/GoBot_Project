#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import argparse
from typing import List, Tuple
import pandas as pd
import numpy as np
import folium

R_EARTH = 6378137.0  # WGS84

# ---------------- 거리 함수 ----------------
def haversine_m(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R_EARTH * c

# ---------------- CSV 로드 ----------------
def load_lat_lon_csv(path: str) -> List[Tuple[float, float]]:
    df = pd.read_csv(path)
    cols = [c.lower().strip() for c in df.columns]

    lat_col = None
    lon_col = None
    for cand in ["lat","latitude","gps_lat","y"]:
        if cand in cols:
            lat_col = df.columns[cols.index(cand)]
            break
    for cand in ["lon","lng","longitude","gps_lon","x"]:
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
def dedup_points(pts: List[Tuple[float,float]], eps_m: float = 0.5) -> List[Tuple[float,float]]:
    out = [pts[0]]
    for p in pts[1:]:
        if haversine_m(out[-1][0], out[-1][1], p[0], p[1]) >= eps_m:
            out.append(p)
    return out

# ---------------- 최근접 인덱스 ----------------
def nearest_idx(pts: List[Tuple[float,float]], lat: float, lon: float) -> int:
    best_i = 0
    best_d = 1e18
    for i,(la,lo) in enumerate(pts):
        d = haversine_m(lat, lon, la, lo)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i

# ---------------- 엣지 유효성 체크 ----------------
def edge_supported(p1, p2, raw_pts, step_m=1.0, max_off_m=3.0) -> bool:
    total_d = haversine_m(p1[0], p1[1], p2[0], p2[1])
    if total_d <= step_m:
        return True

    n_steps = max(2, int(total_d / step_m))
    for k in range(1, n_steps):   # 끝점은 어차피 raw point에 붙어있음
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
def build_graph(nodes, raw_pts, neighbor_radius_m=5.0, step_m=1.0, max_off_m=3.0):
    n = len(nodes)
    adj = [[] for _ in range(n)]

    # 1) 순차 이웃 (i <-> i+1)
    for i in range(n-1):
        p1, p2 = nodes[i], nodes[i+1]
        if edge_supported(p1, p2, raw_pts, step_m, max_off_m):
            d = haversine_m(p1[0], p1[1], p2[0], p2[1])
            adj[i].append((i+1, d))
            adj[i+1].append((i, d))

    # 2) 근접 노드 추가 연결 (짧은 지름길, 단 3m 규칙 통과한 것만)
    for i in range(n):
        for j in range(i+2, n):
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

    path = []
    cur = goal
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path

# ---------------- 메인 ----------------
def main():
    ap = argparse.ArgumentParser(description="수집 경로 밖으로 3m 이상 안 벗어나는 경로 플래너")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--current-lat", required=True, type=float)
    ap.add_argument("--current-lon", required=True, type=float)
    ap.add_argument("--dest-lat", required=True, type=float)
    ap.add_argument("--dest-lon", required=True, type=float)
    ap.add_argument("--out", default="route_constrained.html")
    ap.add_argument("--neighbor-radius-m", type=float, default=5.0)
    ap.add_argument("--max-off-m", type=float, default=3.0)
    ap.add_argument("--dedup-eps-m", type=float, default=0.5)
    args = ap.parse_args()

    # 1) 원본 포인트
    raw_pts = load_lat_lon_csv(args.csv)

    # 2) 다운샘플해서 노드 만들기
    nodes = dedup_points(raw_pts, eps_m=args.dedup_eps_m)

    # 3) 시작/목표 노드를 노드들 위에서 찾기
    start_idx = nearest_idx(nodes, args.current_lat, args.current_lon)
    goal_idx  = nearest_idx(nodes, args.dest_lat, args.dest_lon)

    # 4) 그래프 생성 (3m 규칙 적용)
    adj = build_graph(nodes, raw_pts,
                      neighbor_radius_m=args.neighbor_radius_m,
                      step_m=1.0,
                      max_off_m=args.max_off_m)

    # 5) 최단 경로
    idx_path = dijkstra(adj, start_idx, goal_idx)
    if not idx_path:
        print("[ERROR] 조건 만족하는 경로 없음 (3m 규칙 때문에 다 막혔을 수도 있음)")
        return

    path_pts = [nodes[i] for i in idx_path]

    # 6) 시각화
    center_lat = (args.current_lat + args.dest_lat) / 2.0
    center_lon = (args.current_lon + args.dest_lon) / 2.0
    m = folium.Map(location=[center_lat, center_lon], zoom_start=18)

    # 전체 수집 포인트
    for la, lo in raw_pts:
        folium.CircleMarker([la, lo], radius=1, color="#999999",
                            fill=True, fill_opacity=0.6).add_to(m)

    # 노드(다운샘플)
    for la, lo in nodes:
        folium.CircleMarker([la, lo], radius=2, color="#ff8800",
                            fill=True, fill_opacity=0.8).add_to(m)

    # 최종 경로
    folium.PolyLine(path_pts, color="blue", weight=5, opacity=0.9,
                    tooltip=f"path {len(path_pts)} pts").add_to(m)

    # 시작/목표 마커
    folium.Marker([args.current_lat, args.current_lon],
                  icon=folium.Icon(color="green", icon="play"),
                  tooltip="start input").add_to(m)
    folium.Marker([args.dest_lat, args.dest_lon],
                  icon=folium.Icon(color="red", icon="flag"),
                  tooltip="goal input").add_to(m)

    m.save(args.out)
    print(f"[OK] saved: {args.out}")
    print(f"nodes={len(nodes)}, raw_pts={len(raw_pts)}, path_len={len(path_pts)}")


if __name__ == "__main__":
    main()
