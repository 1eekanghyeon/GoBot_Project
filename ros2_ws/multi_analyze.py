#!/usr/bin/env python3
import rosbag2_py
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

# === 1️⃣ Helper: Bag 로딩 함수 ===
def load_odom_from_bag(bag_path, topic_name):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    xs, ys, vxs, vys = [], [], [], []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, Odometry)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
        vxs.append(msg.twist.twist.linear.x)
        vys.append(msg.twist.twist.linear.y)

    if len(xs) == 0:
        print(f"[WARN] {topic_name} 데이터 없음.")
        return None
    return np.array(xs), np.array(ys), np.array(vxs), np.array(vys)


# === 2️⃣ Helper: 통계 요약 출력 ===
def summarize(name, xs, ys, vxs, vys):
    print(f"\n📊 ===== {name} =====")
    print(f"샘플 수: {len(xs)}")
    print(f"🧩 위치 평균: X={np.mean(xs):.3f}, Y={np.mean(ys):.3f}")
    print(f"🧭 위치 표준편차: X={np.std(xs):.4f}, Y={np.std(ys):.4f}")
    print(f"🚗 속도 평균: Vx={np.mean(vxs):.4f}, Vy={np.mean(vys):.4f}")
    print(f"⚙️  속도 표준편차: Vx={np.std(vxs):.5f}, Vy={np.std(vys):.5f}")

    # 변화율 계산
    dx, dy = np.diff(xs), np.diff(ys)
    dvx, dvy = np.diff(vxs), np.diff(vys)
    move = np.sqrt(dx**2 + dy**2)
    accel = np.sqrt(dvx**2 + dvy**2)

    print(f"📈 평균 이동량: {np.mean(move):.6f} m/frame (max {np.max(move):.6f})")
    print(f"⚡ 평균 속도 변화량: {np.mean(accel):.6f} m/s per frame (max {np.max(accel):.6f})")


# === 3️⃣ Main 실행 ===
if __name__ == "__main__":
    bag_path = "odom_bag"

    topics = {
        "filtered_map": "/odometry/filtered_map",
        "filtered_odom": "/odometry/filtered_odom",
        "gps_raw": "/odometry/gps",
    }

    for name, topic in topics.items():
        data = load_odom_from_bag(bag_path, topic)
        if data:
            summarize(name, *data)

    print("\n✅ 분석 완료.")
    print(" - 위치 표준편차가 작을수록 안정적인 EKF 결과입니다.")
    print(" - 속도 변화율이 작을수록 필터가 부드럽게 작동하고 있습니다.")
