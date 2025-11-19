import asyncio
import json
import subprocess

import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError

# ===== 설정 =====
JETSON_WS_URL = "ws://192.168.252.193:8080"  # 예: "ws://192.168.0.10:8080"
MY_ROLE = "wsl"  # 음성 파이프라인에서 "to"로 찍어줄 역할 이름이랑 맞춰야 함

# 음성 명령 -> ros2 topic pub 명령어 매핑
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


async def handle_ws():
    # 끊기면 알아서 재접속하는 무한 루프
    while True:
        try:
            print("[*] WebSocket 연결 시도...")
            async with websockets.connect(JETSON_WS_URL) as ws:
                # 1) 접속하자마자 내 role 보내기
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
                        # 브로드캐스트 아니고 다른 role이면 무시
                        continue

                    payload = msg.get("payload", "")

                    # payload는 "sit", "stand", "down" 같은 문자열이라고 가정
                    if not isinstance(payload, str):
                        print(f"[i] 문자열 payload 아님: {type(payload)} -> {payload}")
                        continue

                    text = payload.strip().lower()

                    if not text:
                        print("[i] payload가 빈 문자열, 무시")
                        continue

                    print(f"[i] 음성 명령 수신: '{text}'")

                    # ros2 명령 매핑 확인
                    cmd = ROS_CMD_MAP.get(text)
                    if not cmd:
                        print(f"[i] 처리 대상 아님: {text}")
                        continue

                    # ros2 topic pub 실행
                    try:
                        print(f"[CMD] 실행: {' '.join(cmd)}")
                        subprocess.run(cmd, check=True)
                        print("[CMD] 실행 완료")
                    except subprocess.CalledProcessError as e:
                        print(f"[!] ros2 명령 실행 실패: {e}")
                    except Exception as e:
                        print(f"[!] ros2 명령 실행 중 예기치 못한 에러: {e}")

        except OSError as e:
            # 연결 시도 자체가 실패할 때 (서버 안 떠있음, 네트워크 문제 등)
            print(f"[!] WebSocket 연결 실패 (OSError): {e}")
        except Exception as e:
            # 그 외 handle_ws 전체 레벨 에러
            print(f"[!] WebSocket 처리 중 예기치 못한 에러: {e}")

        # 여기까지 왔다는 건 어쨌든 연결이 끊긴 상태
        print("[i] 3초 후 WebSocket 재접속 시도...")
        await asyncio.sleep(3)


def main():
    asyncio.run(handle_ws())


if __name__ == "__main__":
    main()
