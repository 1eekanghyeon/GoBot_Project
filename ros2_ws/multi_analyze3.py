#!/usr/bin/env python3
import argparse
import re
from typing import Dict, List, Tuple, Optional
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message

# ROS2 msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray   # ★ 추가


# ------------------------ Bag helpers ------------------------
def open_reader(bag_path: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def load_odom_from_bag(bag_path: str, topic_name: str):
    xs, ys, vxs, vys = [], [], [], []
    reader = open_reader(bag_path)
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, Odometry)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
        vxs.append(msg.twist.twist.linear.x)
        vys.append(msg.twist.twist.linear.y)
    if not xs:
        print(f"[WARN] {topic_name} 데이터 없음.")
        return None
    return np.array(xs), np.array(ys), np.array(vxs), np.array(vys)


def load_imu_cov8(bag_path: str, imu_topic: str):
    vals = []
    reader = open_reader(bag_path)
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != imu_topic:
            continue
        msg: Imu = deserialize_message(data, Imu)
        cov = list(msg.orientation_covariance)
        if len(cov) == 9:
            vals.append(cov[8])
    if not vals:
        print(f"[WARN] {imu_topic} IMU 데이터 없음.")
        return None
    return np.array(vals)


def _stamp_to_sec(ts) -> float:
    # builtin_interfaces/msg/Time
    return float(ts.sec) + float(ts.nanosec) * 1e-9


def load_tf_pair_series(
    bag_path: str,
    parent: str,
    child: str,
    include_static: bool = True,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    Returns (t, trans_xy_norm, yaw_rad) for the given parent->child TF.
    - We aggregate both /tf and /tf_static (optional).
    """
    times = []
    xy = []
    yaw = []

    # helper to process a TransformStamped
    def handle_ts(ts: TransformStamped):
        if ts.header.frame_id == parent and ts.child_frame_id == child:
            t = _stamp_to_sec(ts.header.stamp)
            tx, ty, tz = ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z
            qx, qy, qz, qw = (
                ts.transform.rotation.x,
                ts.transform.rotation.y,
                ts.transform.rotation.z,
                ts.transform.rotation.w,
            )
            # yaw from quaternion (Z-up)
            siny = 2.0 * (qw * qz + qx * qy)
            cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw_z = np.arctan2(siny, cosy)
            times.append(t)
            xy.append([tx, ty])
            yaw.append(yaw_z)

    # /tf
    reader = open_reader(bag_path)
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != "/tf":
            continue
        msg = deserialize_message(data, TFMessage)
        for ts in msg.transforms:
            handle_ts(ts)

    # /tf_static
    if include_static:
        reader = open_reader(bag_path)
        while reader.has_next():
            topic, data, _t = reader.read_next()
            if topic != "/tf_static":
                continue
            msg = deserialize_message(data, TFMessage)
            for ts in msg.transforms:
                handle_ts(ts)

    if not times:
        print(f"[WARN] TF {parent}->{child} 없음.")
        return None

    t = np.array(times)
    xy = np.array(xy)
    yaw = np.array(yaw)
    # sort by time
    idx = np.argsort(t)
    return t[idx], np.linalg.norm(xy[idx], axis=1), yaw[idx]


def load_diagnostics_from_bag(bag_path: str, topic_name: str = "/diagnostics"):
    """
    /diagnostics에서 DiagnosticArray 읽어서
    [(t, name, level, message, {key: value})] 리스트로 반환
    """
    entries = []
    reader = open_reader(bag_path)
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != topic_name:
            continue
        msg: DiagnosticArray = deserialize_message(data, DiagnosticArray)
        t = _stamp_to_sec(msg.header.stamp)
        for st in msg.status:
            kv = {v.key: v.value for v in st.values}
            entries.append((t, st.name, st.level, st.message, kv))

    if not entries:
        print(f"[WARN] {topic_name} diagnostics 데이터 없음.")
        return None
    return entries


# ------------------------ Stats helpers ------------------------
def summarize_odom(name, xs, ys, vxs, vys):
    print(f"\n📊 ===== {name} =====")
    print(f"샘플 수: {len(xs)}")
    print(f"🧩 위치 평균: X={np.mean(xs):.3f}, Y={np.mean(ys):.3f}")
    print(f"🧭 위치 표준편차: X={np.std(xs):.4f}, Y={np.std(ys):.4f}")
    print(f"🚗 속도 평균: Vx={np.mean(vxs):.4f}, Vy={np.mean(vys):.4f}")
    print(f"⚙️  속도 표준편차: Vx={np.std(vxs):.5f}, Vy={np.std(vys):.5f}")

    dx, dy = np.diff(xs), np.diff(ys)
    dvx, dvy = np.diff(vxs), np.diff(vys)
    move = np.sqrt(dx**2 + dy**2)
    accel = np.sqrt(dvx**2 + dvy**2)

    print(f"📈 평균 이동량: {np.mean(move):.6f} m/frame (max {np.max(move):.6f})")
    print(f"⚡ 평균 속도 변화량: {np.mean(accel):.6f} m/s per frame (max {np.max(accel):.6f})")


def summarize_imu_cov8(name, cov8: np.ndarray):
    zero_ratio = float(np.sum(np.isclose(cov8, 0.0))) / len(cov8)
    nonzero = cov8[~np.isclose(cov8, 0.0)] if np.any(~np.isclose(cov8, 0.0)) else np.array([])
    print(f"\n🧪 ===== {name} (IMU orientation_covariance[8]) =====")
    print(f"샘플 수: {len(cov8)}")
    print(f"0.0 비율: {zero_ratio*100:.1f}%")
    if nonzero.size > 0:
        print(
            f"비영(非0) 통계: mean={np.mean(nonzero):.6g}, std={np.std(nonzero):.6g}, "
            f"min={np.min(nonzero):.6g}, max={np.max(nonzero):.6g}"
        )
        print("권장값 예: (2°)^2 ≈ 0.0012 rad² 이상")
    else:
        print("모든 샘플이 0.0 입니다. (EKF가 yaw를 신뢰하지 못할 수 있음)")


def _wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def summarize_tf_pair(title: str, t: np.ndarray, xy_norm: np.ndarray, yaw: np.ndarray):
    print(f"\n🧭 ===== TF: {title} =====")
    print(f"샘플 수: {len(t)}")
    if len(t) < 2:
        print("표본이 부족합니다.")
        return
    dt = np.diff(t)
    dxy = np.diff(xy_norm)
    dyaw = _wrap_pi(np.diff(yaw))

    print(
        f"평균 xy 노름: {np.mean(xy_norm):.4f} m (std {np.std(xy_norm):.4f}), "
        f"초기→최종 변화량: {(xy_norm[-1]-xy_norm[0]):.4f} m"
    )
    print(f"프레임간 xy 변화 |Δ| 평균: {np.mean(np.abs(dxy)):.6f} m (max {np.max(np.abs(dxy)):.6f})")
    print(f"프레임간 yaw 변화 평균: {np.mean(np.abs(dyaw)):.6f} rad (max {np.max(np.abs(dyaw)):.6f})")
    # per-second rates (rough)
    valid = dt > 0
    if np.any(valid):
        rate_xy = np.abs(dxy[valid] / dt[valid])
        rate_yaw = np.abs(dyaw[valid] / dt[valid])
        print(f"초당 xy 점프 평균: {np.mean(rate_xy):.6f} m/s (max {np.max(rate_xy):.6f})")
        print(f"초당 yaw 점프 평균: {np.mean(rate_yaw):.6f} rad/s (max {np.max(rate_yaw):.6f})")
    else:
        print("타임스탬프 간격(dt)이 0입니다.")


def summarize_ekf_diagnostics(diag_entries):
    """
    robot_localization EKF 관련 diagnostics만 뽑아서 요약
    """
    print("\n🩺 ===== Diagnostics: robot_localization / EKF =====")
    if not diag_entries:
        print("[WARN] diagnostics 데이터 없음.")
        return

    grouped = defaultdict(list)
    for t, name, level, msg, kv in diag_entries:
        if "ekf_filter_node" in name:
            grouped[name].append((t, level, msg, kv))

    if not grouped:
        print("ekf_filter_node* 관련 diagnostics 없음.")
        return

    level_label = {0: "OK", 1: "WARN", 2: "ERROR"}

    for name in sorted(grouped.keys()):
        entries = grouped[name]
        levels = [e[1] for e in entries]
        max_level = max(levels)
        label = level_label.get(max_level, str(max_level))
        t_last, lvl_last, msg_last, kv_last = entries[-1]

        print(f"\n- {name}")
        print(f"  · 샘플 수: {len(entries)}")
        print(f"  · 최악 level: {label} ({max_level})")
        print(f"  · 마지막 message: {msg_last}")

        # odometry/filtered topic status -> 실제 발행 여부/주파수 확인
        if "odometry/filtered topic status" in name:
            ev = kv_last.get("Events in window", "?")
            freq = kv_last.get("Actual frequency (Hz)", "?")
            min_f = kv_last.get("Minimum acceptable frequency (Hz)", "?")
            max_f = kv_last.get("Maximum acceptable frequency (Hz)", "?")
            print(f"  · 최근 윈도우 이벤트: {ev}")
            print(f"  · 주파수: {freq} Hz (허용: {min_f} ~ {max_f} Hz)")
            if "No events recorded" in msg_last:
                print("  ⚠️ odometry/filtered가 한 번도 발행되지 않았습니다.")

        # Filter diagnostic updater -> absolute pose inputs 같은 구조 경고 체크
        if "Filter diagnostic updater" in name:
            cnt_abs = sum("absolute pose inputs" in e[2] for e in entries)
            if cnt_abs > 0:
                print(f"  ⚠️ 'absolute pose inputs' 관련 경고 발생 횟수: {cnt_abs}")
            # 다른 경고 패턴 더 추가하고 싶으면 여기서 msg_last 검사해서 출력하면 됨


# ------------------------ Remap helpers ------------------------
def parse_remaps(remap_list: List[str]) -> Dict[str, str]:
    """
    Accepts ROS-like remap strings: '/imu:=/imu_converted'
    Returns dict {"/imu": "/imu_converted"}
    """
    remaps = {}
    for item in remap_list:
        m = re.match(r'([^:=\s]+)\s*:=\s*([^:=\s]+)$', item.strip())
        if not m:
            raise ValueError(f"--remap 형식 오류: {item!r} (예: --remap /imu:=/imu_converted)")
        src, dst = m.group(1), m.group(2)
        remaps[src] = dst
    return remaps


def apply_remap(name2topic: Dict[str, str], remaps: Dict[str, str]) -> Dict[str, str]:
    """
    name2topic has concrete topic strings. If any value matches a key in remaps, replace.
    Also supports canonical aliases like '/imu' to map 'imu_topic'.
    """
    out = dict(name2topic)
    for k, v in list(out.items()):
        # value-based remap
        if v in remaps:
            out[k] = remaps[v]
    return out


# ------------------------ Main ------------------------
def main():
    ap = argparse.ArgumentParser(description="Odometry/TF/IMU/Diagnostics quick analyzer")
    ap.add_argument("--bag", default="odom_bag", help="ros2 bag 디렉토리 (기본: odom_bag)")
    # topics
    ap.add_argument("--filtered_map", default="/odometry/filtered_map")
    ap.add_argument("--filtered_odom", default="/odometry/filtered_odom")
    ap.add_argument("--gps", default="/odometry/gps")
    ap.add_argument("--imu", default="/imu/gps_heading", help="IMU 토픽(orientation_covariance[8] 점검)")
    ap.add_argument("--diagnostics", default="/diagnostics", help="diagnostics 토픽 이름 (기본: /diagnostics)")
    # tf pairs
    ap.add_argument(
        "--tf-pair",
        action="append",
        default=["map:odom", "odom:base_link"],
        help="요약할 TF parent:child (여러 번 지정 가능). 기본: map:odom, odom:base_link",
    )
    # remap
    ap.add_argument(
        "--remap",
        action="append",
        default=[],
        help="ROS 스타일 리맵 (예: --remap /imu:=/imu_converted) 여러개 가능",
    )
    args = ap.parse_args()

    # assemble topics
    topics = {
        "filtered_map": args.filtered_map,
        "filtered_odom": args.filtered_odom,
        "gps_raw": args.gps,
        "imu": args.imu,
        "diagnostics": args.diagnostics,
        "/tf": "/tf",
        "/tf_static": "/tf_static",
    }

    # apply remaps like /imu:=/imu_converted
    remaps = parse_remaps(args.remap) if args.remap else {}
    topics = apply_remap(topics, remaps)

    # 1) Odom summaries
    for name_key in ("filtered_map", "filtered_odom", "gps_raw"):
        data = load_odom_from_bag(args.bag, topics[name_key])
        if data:
            summarize_odom(name_key, *data)

    # 2) IMU covariance[8] check (optional if topic exists)
    cov8 = load_imu_cov8(args.bag, topics["imu"])
    if cov8 is not None:
        summarize_imu_cov8(topics["imu"], cov8)

    # 3) TF summaries
    tf_pairs: List[Tuple[str, str]] = []
    for spec in args.tf_pair:
        if ":" not in spec:
            print(f"[WARN] --tf-pair 형식 무시: {spec!r}")
            continue
        p, c = spec.split(":", 1)
        tf_pairs.append((p.strip(), c.strip()))

    for p, c in tf_pairs:
        series = load_tf_pair_series(args.bag, p, c, include_static=True)
        if series:
            t, xy_norm, yaw = series
            summarize_tf_pair(f"{p}->{c}", t, xy_norm, yaw)

    # 4) Diagnostics (robot_localization 중심)
    diag_entries = load_diagnostics_from_bag(args.bag, topics["diagnostics"])
    if diag_entries is not None:
        summarize_ekf_diagnostics(diag_entries)

    print("\n✅ 분석 완료.")
    print(" - 위치 표준편차가 작을수록 안정적인 EKF 결과입니다.")
    print(" - 전역 TF(map→odom) 변화량이 작을수록 map 프레임이 안정적입니다.")
    print(" - IMU orientation_covariance[8]가 0이 아니어야(EKF가 yaw를 신뢰) 합니다.")


if __name__ == "__main__":
    main()
