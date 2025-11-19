#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import json
import signal
import subprocess
from typing import Optional

import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError

# ===== 설정 =====
JETSON_WS_URL = "ws://192.168.0.21:8080"  # 서버 주소 맞게 수정
MY_ROLE = "wsl"                              # 서버에서 "to": "wsl" 로 보내는 메시지 처리

# ===== 음성 명령 -> ros2 topic pub 매핑 =====
ROS_CMD_MAP = {
    "sit": [
        "ros2", "topic", "pub", "--once",
        "/webrtc_req", "go2_interfaces/msg/WebRtcReq",
        "{id: 0, topic: 'rt/api/sport/request', api_id: 1009, parameter: '', priority: 0}",
    ],
    "stand": [
        "ros2", "topic", "pub", "--once",
        "/webrtc_req", "go2_interfaces/msg/WebRtcReq",
        "{id: 0, topic: 'rt/api/sport/request', api_id: 1004, parameter: '', priority: 0}",
    ],
    "down": [
        "ros2", "topic", "pub", "--once",
        "/webrtc_req", "go2_interfaces/msg/WebRtcReq",
        "{id: 0, topic: 'rt/api/sport/request', api_id: 1005, parameter: '', priority: 0}",
    ],
    "hand": [
        "ros2", "topic", "pub", "--once",
        "/webrtc_req", "go2_interfaces/msg/WebRtcReq",
        "{id: 0, topic: 'rt/api/sport/request', api_id: 1016, parameter: '{}', priority: 0}",
    ],
}

# ===== nav / distance 상태 =====
_current_nav_proc: Optional[subprocess.Popen] = None  # gps_rtk_pid_nav 프로세스 핸들
last_distance: Optional[float] = None
paused: bool = True               # 기본은 정지 상태
last_pause_sent: Optional[bool] = None  # 같은 값이면 /nav_pause 다시 안 보냄


# ============ 공통 bash 실행 헬퍼 ============

def run_bash(script: str) -> None:
    """bash -lc 로 멀티라인 스크립트 실행."""
    print(f"\n[CMD] 실행 ===\n{script}\n============\n")
    subprocess.run(["bash", "-lc", script], check=True)


# ============ /nav_pause 제어 (PID 쪽 준비용) ============

def send_nav_pause(paused_flag: bool) -> None:
    """
    gps_rtk_pid_nav 에 /nav_pause Bool 메시지 보내기.
    - 지금은 PID 노드에서 안 받으면 그냥 무시됨.
    - 나중에 gps_rtk_pid_nav.py에 구독만 붙이면 바로 동작.
    """
    global last_pause_sent

    if last_pause_sent is not None and last_pause_sent == paused_flag:
        return  # 상태 변화 없으면 패스

    last_pause_sent = paused_flag
    val = "true" if paused_flag else "false"
    print(f"[PAUSE] /nav_pause -> {val}")

    script = f"""
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /nav_pause std_msgs/msg/Bool "data: {val}" --once
"""
    try:
        run_bash(script)
    except Exception as e:
        print(f"[PAUSE] /nav_pause 발행 실패: {e}")


# ============ gps_rtk_pid_nav 프로세스 관리 ============

def stop_nav_process() -> None:
    """
    gps_rtk_pid_nav 프로세스 완전 종료.
    - 새 목적지로 다시 시작할 때만 사용.
    - 거리로 멈추는 건 /nav_pause 로만 처리할 예정.
    """
    global _current_nav_proc

    if _current_nav_proc is None:
        return

    proc = _current_nav_proc
    if proc.poll() is not None:
        _current_nav_proc = None
        return

    print(f"[NAV] 기존 gps_rtk_pid_nav 종료 시도 (PID={proc.pid})")
    try:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print("[NAV] SIGINT로 안 죽어서 SIGTERM 전송")
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                print("[NAV] SIGTERM도 안 먹어서 SIGKILL 전송")
                proc.kill()
    except Exception as e:
        print(f"[NAV] 기존 프로세스 종료 중 예외: {e}")

    _current_nav_proc = None


def start_gps_pid_nav(lat: float, lon: float) -> None:
    """
    type=='nav' 메시지로 받은 lat/lon 으로 gps_rtk_pid_nav 실행.
    - 기존 프로세스는 종료 후 새로 시작.
    - /cmd_vel_out 으로 바로 속도 뿜도록 설정.
    """
    global _current_nav_proc, paused

    stop_nav_process()

    script = f"""
cd ~/ros2_ws
unset ROS_LOCALHOST_ONLY
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run gps_heading_init gps_rtk_pid_nav \\
  --ros-args \\
  -p fix_topic:=/fix \\
  -p cmd_vel_topic:=/cmd_vel_out \\
  -p csv_path:=/home/gobot/ros2_ws/src/GoBot_Project/src/gps_heading_init/data/gps_data.csv \\
  -p dest_lat:={lat} \\
  -p dest_lon:={lon} \\
  -p use_fix_as_start:=true \\
  -p neighbor_radius_m:=5.0 \\
  -p max_off_m:=3.0 \\
  -p dedup_eps_m:=0.5 \\
  -p init_straight_distance:=3.0 \\
  -p init_straight_speed:=0.2
"""

    print("\n[NAV] gps_rtk_pid_nav 새 목표로 실행 ===")
    print(script)
    print("=======================================\n")

    proc = subprocess.Popen(
        ["bash", "-lc", script],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    _current_nav_proc = proc
    print(f"[NAV] gps_rtk_pid_nav 프로세스 시작 (PID={proc.pid})")

    # 현재 paused 상태에 맞춰 /nav_pause 쏴주기 (PID 코드 나중에 붙이면 바로 동작)
    send_nav_pause(paused)


async def handle_nav_message(payload: dict) -> None:
    """
    {"type":"nav","to":"wsl","payload":{"lat": ..., "lon": ...}}
    형식 처리.
    """
    lat = payload.get("lat")
    lon = payload.get("lon")

    try:
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        print(f"[NAV] lat/lon 변환 실패: {payload}")
        return

    print(f"[NAV] nav 요청 수신: lat={lat}, lon={lon}, paused={paused}, last_distance={last_distance}")
    await asyncio.to_thread(start_gps_pid_nav, lat, lon)
    print("[NAV] gps_rtk_pid_nav 실행 요청 완료")


# ============ distance 메시지 처리 ============

async def handle_distance_message(payload) -> None:
    """
    {"type":"distance","from":"camera","to":"wsl","payload":{"distance":3.947}}
    처리.

    - distance > 4.0  → paused=True  → /nav_pause True  (멈춰서 대기)
    - distance <= 4.0 → paused=False → /nav_pause False (기존 경로 그대로 재개)
    """
    global last_distance, paused

    d = payload
    try:
        d = float(d)
    except (TypeError, ValueError):
        print(f"[DIST] distance 변환 실패: {payload}")
        return

    last_distance = d
    print(f"[DIST] 새 거리 수신: {d:.3f} m")

    if d > 4.0:
        if not paused:
            print("[DIST] distance > 4.0 → 일시정지 모드로 전환")
        paused = True
        await asyncio.to_thread(send_nav_pause, True)
    else:
        if paused:
            print("[DIST] distance <= 4.0 → 주행 재개 모드로 전환")
        paused = False
        await asyncio.to_thread(send_nav_pause, False)


# ============ 음성 명령 처리 ============

async def handle_voice_message(payload: str) -> None:
    text = payload.strip().lower()
    if not text:
        print("[VOICE] 빈 문자열 payload, 무시")
        return

    print(f"[VOICE] 음성 명령 수신: '{text}'")

    cmd = ROS_CMD_MAP.get(text)
    if not cmd:
        print(f"[VOICE] 처리 대상 아님: {text}")
        return

    try:
        print(f"[CMD] 실행: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        print("[CMD] 실행 완료")
    except subprocess.CalledProcessError as e:
        print(f"[!] ros2 명령 실행 실패: {e}")
    except Exception as e:
        print(f"[!] ros2 명령 실행 중 예기치 못한 에러: {e}")


# ============ WebSocket 메인 루프 ============

async def handle_ws():
    global paused

    # 처음에는 안전하게 멈춘 상태로 시작
    paused = True

    while True:
        try:
            print("[*] WebSocket 연결 시도...", JETSON_WS_URL)
            async with websockets.connect(JETSON_WS_URL) as ws:
                # 1) 접속하자마자 role 전송
                hello = {"role": MY_ROLE}
                await ws.send(json.dumps(hello))
                print(f"[+] 서버 접속. role='{MY_ROLE}' 전송 완료")

                # 2) 메시지 수신 루프
                while True:
                    try:
                        raw = await ws.recv()
                    except (ConnectionClosedOK, ConnectionClosedError) as e:
                        print(f"[!] WebSocket 연결 종료: {e}")
                        break
                    except Exception as e:
                        print(f"[!] ws.recv() 에러: {e}")
                        break

                    print(f"[WS 수신 RAW] {raw}")

                    try:
                        msg = json.loads(raw)
                    except Exception as e:
                        print(f"[!] JSON decode 실패: {e}")
                        continue

                    # 나에게 온 메시지만 처리 (to 필터)
                    to_role = msg.get("to")
                    if to_role not in (None, MY_ROLE):
                        continue

                    msg_type = msg.get("type")
                    payload = msg.get("payload", "")

                    # --- 1) distance ---
                    if msg_type == "distance":
                        await handle_distance_message(payload)
                        continue

                    # --- 2) nav (반드시 type == "nav" 일 때만) ---
                    if msg_type == "nav":
                        if not isinstance(payload, dict):
                            print(f"[NAV] payload 형식 이상: {payload}")
                            continue
                        await handle_nav_message(payload)
                        continue

                    # --- 3) 그 외: payload가 문자열이면 음성 명령으로 간주 ---
                    if isinstance(payload, str):
                        await handle_voice_message(payload)
                        continue

                    print(f"[i] 처리하지 않는 메시지 type={msg_type}, payload={payload}")

        except OSError as e:
            print(f"[!] WebSocket 연결 실패 (OSError): {e}")
        except Exception as e:
            print(f"[!] WebSocket 처리 중 예기치 못한 에러: {e}")

        print("[i] 3초 후 WebSocket 재접속 시도...")
        await asyncio.sleep(3)


def main():
    asyncio.run(handle_ws())


if __name__ == "__main__":
    main()
