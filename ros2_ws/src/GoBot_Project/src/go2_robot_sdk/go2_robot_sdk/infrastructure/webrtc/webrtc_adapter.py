# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
import io
import base64
import time  # <-- NEW: 상태머신 타이밍용

from typing import Callable, Dict, Any

from pydub import AudioSegment

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotData, RobotConfig
from .go2_connection import Go2Connection
from ...application.utils.command_generator import gen_command, gen_mov_command
from ...domain.constants import ROBOT_CMD, RTC_TOPIC

logger = logging.getLogger(__name__)


class WebRTCAdapter(IRobotDataReceiver, IRobotController):
    """WebRTC adapter for robot communication"""

    def __init__(
        self,
        config: RobotConfig,
        on_validated_callback: Callable,
        on_video_frame_callback: Callable = None,
        event_loop=None
    ):
        self.config = config
        self.connections: Dict[str, Go2Connection] = {}
        self.data_callback: Callable[[RobotData], None] = None

        # 일반 webrtc API 요청 (ex. obstacle avoidance 토글 등)이 쌓이는 곳
        self.webrtc_msgs = asyncio.Queue()

        # ★ NEW: 오디오 스트리밍 상태 저장 (로봇별로 1개만)
        # robot_id -> {
        #   "stage": "start" | "chunks" | "wait" | "end",
        #   "chunks": [ "b64chunk0", "b64chunk1", ... ],
        #   "next_idx": 0,
        #   "total": <int>,
        #   "duration_sec": <float>,
        #   "last_send": <time.monotonic()>,
        #   "send_interval": 0.3,
        #   "playback_start": <time.monotonic() or None>,
        # }
        self.pending_audio: Dict[str, Dict[str, Any]] = {}  # <-- NEW

        self.on_validated_callback = on_validated_callback
        self.on_video_frame_callback = on_video_frame_callback

        # event loop 저장
        if event_loop:
            self.main_loop = event_loop
        else:
            try:
                self.main_loop = asyncio.get_running_loop()
            except RuntimeError:
                self.main_loop = None

    async def connect(self, robot_id: str) -> None:
        """Connect to robot via WebRTC"""
        try:
            robot_idx = int(robot_id)
            robot_ip = self.config.robot_ip_list[robot_idx]

            conn = Go2Connection(
                robot_ip=robot_ip,
                robot_num=robot_id,
                token=self.config.token,
                on_validated=self._on_validated,
                on_message=self._on_data_channel_message,
                on_video_frame=self.on_video_frame_callback if self.config.enable_video else None,
                decode_lidar=self.config.decode_lidar,
            )

            self.connections[robot_id] = conn
            await conn.connect()
            await conn.disableTrafficSaving(True)

            logger.info(f"Connected to robot {robot_id} at {robot_ip}")

        except Exception as e:
            logger.error(f"Failed to connect to robot {robot_id}: {e}")
            raise

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot"""
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'disconnect'):
                    await connection.disconnect()
                elif hasattr(connection, 'pc') and connection.pc:
                    await connection.pc.close()
                del self.connections[robot_id]
                logger.info(f"Disconnected from robot {robot_id}")
            except Exception as e:
                logger.error(f"Error disconnecting from robot {robot_id}: {e}")

    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Set callback for data reception"""
        self.data_callback = callback

    def _get_or_create_event_loop(self):
        """Get existing event loop or return the main loop"""
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return self.main_loop

    async def _async_send_command(self, connection, command: str):
        """Async wrapper for sending commands"""
        try:
            if hasattr(connection, 'data_channel') and connection.data_channel:
                connection.data_channel.send(command)
        except Exception as e:
            logger.error(f"Error in async send command: {e}")

    def send_command(self, robot_id: str, command: str) -> None:
        """
        실제 WebRTC data_channel 로 JSON 문자열을 쏘는 로우레벨 함수
        """
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'data_channel') and connection.data_channel:
                    loop = self._get_or_create_event_loop()

                    if loop and loop.is_running():
                        asyncio.run_coroutine_threadsafe(
                            self._async_send_command(connection, command),
                            loop
                        )
                    else:
                        # fallback (같은 쓰레드면 그냥 보내기)
                        connection.data_channel.send(command)

                    logger.debug(
                        f"[webrtc send] robot {robot_id} -> {command[:120]}..."
                    )
                else:
                    logger.warning(
                        f"No data channel available for robot {robot_id}"
                    )
            except Exception as e:
                logger.error(f"Error sending command to robot {robot_id}: {e}")

    # ──────────────────────────────
    # Movement / posture commands
    # ──────────────────────────────
    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        try:
            command = gen_mov_command(
                round(x, 2),
                round(y, 2),
                round(z, 2),
                self.config.obstacle_avoidance
            )
            self.send_command(robot_id, command)
        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        try:
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.send_command(robot_id, stand_up_cmd)

            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.send_command(robot_id, move_cmd)
        except Exception as e:
            logger.error(f"Error sending stand up command: {e}")

    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        try:
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.send_command(robot_id, stand_down_cmd)
        except Exception as e:
            logger.error(f"Error sending stand down command: {e}")

    # ──────────────────────────────
    # Generic WebRTC API request
    #   (예: obstacle avoidance on/off)
    # ──────────────────────────────
    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: Any, topic: str) -> None:
        """Enqueue generic WebRTC request (non-audio)"""
        try:
            payload = gen_command(api_id, parameter, topic)
            self.webrtc_msgs.put_nowait(payload)
            logger.debug(f"WebRTC request queued for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error sending WebRTC request: {e}")

    def process_webrtc_commands(self, robot_id: str) -> None:
        """
        Flush queued non-audio WebRTC commands immediately.
        This will be called every loop tick.
        """
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.send_command(robot_id, message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    # ──────────────────────────────
    # Connection validated callback
    # ──────────────────────────────
    def _on_validated(self, robot_id: str) -> None:
        """Callback after connection validation"""
        try:
            if robot_id in self.connections:
                # 로봇 측 topic들 subscribe 걸기
                for topic in RTC_TOPIC.values():
                    sub_msg = json.dumps({"type": "subscribe", "topic": topic})
                    self.connections[robot_id].data_channel.send(sub_msg)

            if self.on_validated_callback:
                self.on_validated_callback(robot_id)

        except Exception as e:
            logger.error(f"Error in validated callback: {e}")

    # ──────────────────────────────
    # Incoming data from robot
    # ──────────────────────────────
    def _on_data_channel_message(self, _, msg: Dict[str, Any], robot_id: str) -> None:
        """Handle incoming data channel messages"""
        try:
            if self.data_callback:
                robot_data = RobotData(robot_id=robot_id, timestamp=0.0)
                # 실제 파싱/퍼블리시는 RobotDataService 쪽에서 함
                self.data_callback(msg, robot_id)
        except Exception as e:
            logger.error(f"Error processing data channel message: {e}")

    # ─────────────────────────────────────────
    # INTERNAL: MP3 -> WAV 변환
    # ─────────────────────────────────────────

    def is_audio_active(self, robot_id: str) -> bool:
        """지금 이 로봇에 업로드/재생 중인 오디오가 있는지 여부"""
        return robot_id in self.pending_audio


    def _mp3_to_wav_bytes(self, mp3_bytes: bytes) -> bytes:
        """
        로봇이 재생 가능한 PCM WAV로 변환.
        실패하면 mp3 그대로라도 리턴해서 최소한 전송은 하게 만든다.
        """
        try:
            audio = AudioSegment.from_file(io.BytesIO(mp3_bytes), format="mp3")
            # Go2 예제가 보통 44.1kHz stereo PCM을 전제했으니까 거기에 맞춘다.
            audio = audio.apply_gain(-8.0)
            audio = audio.set_frame_rate(22050).set_sample_width(2).set_channels(1)

            wav_buf = io.BytesIO()
            audio.export(wav_buf, format="wav")
            return wav_buf.getvalue()

        except Exception as e:
            logger.error(f"[TTS] Failed to convert MP3 to WAV: {e}")
            return mp3_bytes  # fallback

    # ─────────────────────────────────────────
    # PUBLIC: TTS 오디오 재생 예약 (큐잉만)
    # RobotControlService.handle_tts_audio() -> 여기 들어옴
    # ─────────────────────────────────────────

    def _send_audio_cmd(self, robot_id: str, command: str) -> bool:
        """
        오디오 전송용 안전 send.
        - data_channel이 진짜 open인지 확인하고
        - send()가 예외 없이 성공했을 때만 True 리턴
        - 실패하면 False 리턴 (그럼 tick_audio()가 stage를 그대로 유지하고 다음 루프에서 재시도)
        """
        conn = self.connections.get(robot_id)
        if conn is None:
            logger.warning(f"[audio send] robot {robot_id}: no connection")
            return False

        dc = getattr(conn, "data_channel", None)
        if dc is None:
            logger.warning(f"[audio send] robot {robot_id}: no data_channel yet")
            return False

        # aiortc.RTCDataChannel 은 readyState == "open" 일 때만 안정적으로 보낼 수 있어
        state = getattr(dc, "readyState", getattr(dc, "ready_state", "unknown"))
        if state != "open":
            logger.warning(f"[audio send] robot {robot_id}: data_channel not open (state={state})")
            return False

        try:
            dc.send(command)
            logger.debug(f"[audio send] robot {robot_id} -> {command[:120]}...")
            return True
        except Exception as e:
            logger.error(f"[audio send] robot {robot_id} EXC while sending: {e}")
            return False

    def play_audio_track(self, robot_id: str, mp3_bytes: bytes, duration_sec: float) -> None:
        """
        예전 코드에서는 여기서 바로 webrtc_msgs에 4001/4003/4002 몰아넣었지?
        그 방식은 너무 빠르게 쏴서 로봇이 못 따라갈 수 있음.
        -> 바꾼다. 'pending_audio'에 상태로만 저장하고,
           실제 송신은 tick_audio()가 조금씩 한다.
        """
        if self.is_audio_active(robot_id):
            logger.info(f"[TTS] robot {robot_id} is busy, drop new audio")
            return

        try:
            if robot_id not in self.connections:
                logger.error(f"[TTS] play_audio_track: robot {robot_id} not connected")
                return

            # 1) mp3 -> wav (PCM)
            wav_bytes = self._mp3_to_wav_bytes(mp3_bytes)

            # 2) base64 인코딩 (문자열)
            b64_str = base64.b64encode(wav_bytes).decode("utf-8")

            # 3) 16KB 청크로 쪼갠다 (기존 TTS node랑 동일 사이즈)
            chunk_size = 28672         
            b64_bytes = b64_str.encode("utf-8")
            chunk_list = [
                b64_bytes[i:i + chunk_size].decode("utf-8")
                for i in range(0, len(b64_bytes), chunk_size)
            ]

            # pending_audio에 상태머신 초기값 세팅
            self.pending_audio[robot_id] = {
                "stage": "start",
                "chunks": chunk_list,
                "next_idx": 0,
                "total": len(chunk_list),
                "duration_sec": float(duration_sec),
                "last_send": 0.0,
                "send_interval": 0.27,        # 300ms마다 한 번씩만 전송
                "playback_start": None,       # wait 단계에서 재생 대기 시간 측정 시작
            }

            logger.info(
                f"[TTS] Prepared audio for robot {robot_id}: "
                f"{len(chunk_list)} chunks, ~{duration_sec:.2f}s"
            )
            logger.info(f"[TTS] WAV bytes={len(wav_bytes):,}  chunks={len(chunk_list)}  chunk_size={chunk_size}")


        except Exception as e:
            logger.error(f"[TTS] play_audio_track() failed for robot {robot_id}: {e}")

    # ─────────────────────────────────────────
    # NEW: tick_audio()
    # run_robot_control_loop()가 주기적으로 부를 거임.
    #
    #   start 단계: 4001 전송 (재생 시작 알림)
    #   chunks 단계: 4003으로 청크들 순차 전송
    #   wait 단계: 실제 길이(duration_sec)만큼 기다림
    #   end 단계: 4002 전송 (재생 종료 알림)
    #
    # 로봇 펌웨어가 이 순서를 기대한다고 가정.
    # ─────────────────────────────────────────
    def tick_audio(self, robot_id: str) -> None:
        pa = self.pending_audio.get(robot_id)
        if not pa:
            return  # 현재 보낼 오디오 없음

        now = time.monotonic()
        stage = pa["stage"]

        # 전송 rate 제한용 헬퍼
        def throttle_ok():
            # start / chunks / end 단계에서는 0.3초 간격 지켜
            if stage in ("start", "chunks", "end"):
                if (now - pa["last_send"]) < pa["send_interval"]:
                    return False
            return True

        # 1) START 단계 -> api_id=4001 비우고 보내기
                # 1) START 단계 -> api_id=4001 비우고 보내기
        if stage == "start":
            if not throttle_ok():
                return

            start_cmd = gen_command(
                4001,
                "",  # 빈 파라미터
                RTC_TOPIC["AUDIO_HUB_REQ"]
            )

            ok = self._send_audio_cmd(robot_id, start_cmd)
            if ok:
                # 진짜 전송됐을 때만 다음 단계로 넘어간다
                pa["stage"] = "chunks"
                pa["last_send"] = now
                logger.debug(f"[TTS] Sent START to robot {robot_id}")
            else:
                # 전송 실패하면 stage 그대로 'start' 유지해서
                # 다음 tick에서 다시 시도
                logger.info(f"[TTS] START send failed, retrying for robot {robot_id}")
            return

        # 2) CHUNKS 단계 -> api_id=4003로 한 덩어리씩
        if stage == "chunks":
            if not throttle_ok():
                return

            if pa["next_idx"] < pa["total"]:
                idx = pa["next_idx"]

                audio_block = {
                    "current_block_index": idx + 1,
                    "total_block_number": pa["total"],
                    "block_content": pa["chunks"][idx],
                }

                chunk_cmd = gen_command(
                    4003,
                    json.dumps(audio_block),
                    RTC_TOPIC["AUDIO_HUB_REQ"]
                )

                ok = self._send_audio_cmd(robot_id, chunk_cmd)
                if ok:
                    # 성공한 경우에만 다음 청크로 진행
                    pa["next_idx"] += 1
                    pa["last_send"] = now

                    if (idx + 1) % 10 == 0 or (idx + 1) == pa["total"]:
                        logger.info(
                            f"[TTS] Sent chunk {idx+1}/{pa['total']} "
                            f"to robot {robot_id}"
                        )
                else:
                    # 실패하면 같은 idx를 유지 -> 같은 청크를 다음 tick에서 재전송
                    logger.debug(
                        f"[TTS] Chunk {idx+1}/{pa['total']} send failed, "
                        f"retry next tick for robot {robot_id}"
                    )
                return

            else:
                # 모든 청크가 성공적으로 전송 끝났다고 가정
                pa["stage"] = "wait"
                pa["playback_start"] = now
                logger.debug(
                    f"[TTS] All chunks queued OK for robot {robot_id}, "
                    f"waiting {pa['duration_sec']:.2f}s for playback"
                )
                return

        # 3) WAIT 단계 -> duration_sec 만큼 그냥 기다림
        if stage == "wait":
            elapsed = now - (pa["playback_start"] or now)
            if elapsed >= pa["duration_sec"]:
                pa["stage"] = "end"
            return

        # 4) END 단계 -> api_id=4002 보내고 상태 정리
        if stage == "end":
            if not throttle_ok():
                return

            end_cmd = gen_command(
                4002,
                "",
                RTC_TOPIC["AUDIO_HUB_REQ"]
            )

            ok = self._send_audio_cmd(robot_id, end_cmd)
            if ok:
                pa["last_send"] = now
                logger.info(f"[TTS] Sent END to robot {robot_id}, playback done")

                # 전송까지 확실히 됐을 때만 상태 삭제
                del self.pending_audio[robot_id]
            else:
                # 실패하면 stage 유지해서 다음 tick에서 다시 END 보내기
                logger.debug(
                    f"[TTS] END send failed, retry next tick for robot {robot_id}"
                )
            return

