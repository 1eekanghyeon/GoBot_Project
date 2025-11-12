import yaml
import numpy as np

def load_yaml(path):
    with open(path) as f:
        docs = list(yaml.safe_load_all(f))
    xs, ys, vxs, vys = [], [], [], []
    for msg in docs:
        try:
            pose = msg["pose"]["pose"]["position"]
            twist = msg["twist"]["twist"]["linear"]
            xs.append(pose["x"])
            ys.append(pose["y"])
            vxs.append(twist["x"])
            vys.append(twist["y"])
        except Exception:
            continue
    if not xs:
        return None
    return np.array(xs), np.array(ys), np.array(vxs), np.array(vys)


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


# === 파일 경로 설정 ===
datasets = {
    "filtered_map": "odom_map.yaml",
    "filtered_odom": "odom_odom.yaml",
    "gps_raw": "odom_gps.yaml"
}

for name, path in datasets.items():
    try:
        data = load_yaml(path)
        if data is None:
            print(f"{name}: 데이터 없음 또는 형식 오류.")
            continue
        summarize(name, *data)
    except FileNotFoundError:
        print(f"{path} 파일 없음 — 스킵함.")
