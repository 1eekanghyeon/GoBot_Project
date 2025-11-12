#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Enhanced TTS Node (integrated with Go2Driver audio pipeline)

동작 방식:
- /tts (std_msgs/String) 로 "말하고 싶은 문장"이 들어오면
- ElevenLabs로 음성(mp3)을 만든다 (캐시 지원)
- mp3 전체를 base64로 인코딩하고 재생 길이(sec)를 계산한다
- 그 정보를 /tts_audio_stream (std_msgs/String) 으로 publish 한다

Go2DriverNode 가 /tts_audio_stream 을 구독해서
-> RobotControlService.handle_tts_audio()
-> WebRTCAdapter.play_audio_track()
-> 실제 로봇 스피커로 전송 (4001/4003/4002 over WebRTC)
"""

import base64
import io
import json
import os
import time
import hashlib
import threading
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
from enum import Enum

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pydub import AudioSegment
from pydub.playback import play


class AudioFormat(Enum):
    MP3 = "mp3"
    WAV = "wav"
    OGG = "ogg"


class TTSProvider(Enum):
    ELEVENLABS = "elevenlabs"
    GOOGLE = "google"
    AMAZON = "amazon"
    OPENAI = "openai"


@dataclass
class TTSConfig:
    # 너 실제 키/보이스 하드코딩할 거면 여기 그대로 써도 됨
    api_key: str = "sk_c89268251cfb576e8d8dc8716b9cb720b7cc7732d31a7231"
    provider: TTSProvider = TTSProvider.ELEVENLABS
    voice_name: str = "XrExE9yKIg1WjnnlVkGX"

    # 동작 옵션
    local_playback: bool = False          # True면 내 PC에서 바로 스피커 재생
    use_cache: bool = True                # 같은 문장은 캐시 mp3 재사용
    cache_dir: str = "tts_cache"
    audio_quality: str = "standard"
    language: str = "en"

    # ElevenLabs 옵션
    stability: float = 0.97
    similarity_boost: float = 0.8
    model_id: str = "eleven_turbo_v2_5"


class AudioCache:
    """간단한 mp3 캐시 (text+voice+provider => mp3 bytes)"""
    def __init__(self, cache_dir: str, enabled: bool = True):
        self.cache_dir = cache_dir
        self.enabled = enabled
        self._lock = threading.Lock()

        if self.enabled:
            os.makedirs(self.cache_dir, exist_ok=True)

    def _cache_path(self, text: str, voice_name: str, provider: str) -> str:
        key = f"{text}_{voice_name}_{provider}"
        text_hash = hashlib.md5(key.encode()).hexdigest()
        return os.path.join(self.cache_dir, f"{text_hash}.mp3")

    def get(self, text: str, voice_name: str, provider: str) -> Optional[bytes]:
        if not self.enabled:
            return None
        with self._lock:
            p = self._cache_path(text, voice_name, provider)
            if os.path.exists(p):
                with open(p, "rb") as f:
                    return f.read()
        return None

    def put(self, text: str, voice_name: str, provider: str, audio_data: bytes) -> bool:
        if not self.enabled or not audio_data:
            return False
        with self._lock:
            try:
                p = self._cache_path(text, voice_name, provider)
                with open(p, "wb") as f:
                    f.write(audio_data)
                return True
            except Exception:
                return False


class TTSProvider_ElevenLabs:
    """ElevenLabs 호출 래퍼"""
    def __init__(self, config: TTSConfig):
        self.config = config
        self.base_url = "https://api.elevenlabs.io/v1"

    def synthesize(self, text: str) -> Optional[bytes]:
        """
        text -> MP3 bytes
        """
        url = f"{self.base_url}/text-to-speech/{self.config.voice_name}"

        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": self.config.api_key,
        }

        data = {
            "text": text,
            "model_id": self.config.model_id,
            "voice_settings": {
                "stability": self.config.stability,
                "similarity_boost": self.config.similarity_boost
            },
        }

        try:
            resp = requests.post(url, json=data, headers=headers, timeout=30)
            resp.raise_for_status()
            return resp.content  # raw mp3 bytes
        except requests.exceptions.RequestException:
            return None


class AudioInspector:
    """
    길이 계산 같은 post-processing 유틸
    """
    @staticmethod
    def estimate_duration_sec_from_mp3(mp3_bytes: bytes) -> float:
        """
        mp3 바이트 길이를 pydub으로 열어서 초 단위 길이를 추정
        """
        try:
            audio = AudioSegment.from_file(io.BytesIO(mp3_bytes), format="mp3")
            return len(audio) / 1000.0
        except Exception:
            return 0.0

    @staticmethod
    def local_play(mp3_bytes: bytes) -> None:
        """테스트용: 내 PC 스피커에서 재생"""
        try:
            audio = AudioSegment.from_file(io.BytesIO(mp3_bytes), format="mp3")
            play(audio)
        except Exception:
            pass


class EnhancedTTSNode(Node):
    """
    최종 TTS 노드.
    - /tts 구독
    - /tts_audio_stream 퍼블리시
    """

    def __init__(self):
        super().__init__("tts_node")

        # 파라미터 선언 (필요하면 여기 하드코딩 가능)
        self.declare_parameter("api_key", "sk_c89268251cfb576e8d8dc8716b9cb720b7cc7732d31a7231")
        self.declare_parameter("voice_name", "XrExE9yKIg1WjnnlVkGX")
        self.declare_parameter("local_playback", False)
        self.declare_parameter("use_cache", True)

        # 설정 로드
        self.config = TTSConfig(
            api_key=self.get_parameter("api_key").get_parameter_value().string_value,
            voice_name=self.get_parameter("voice_name").get_parameter_value().string_value,
            local_playback=self.get_parameter("local_playback").get_parameter_value().bool_value,
            use_cache=self.get_parameter("use_cache").get_parameter_value().bool_value,
        )

        # 컴포넌트 준비
        self.cache = AudioCache(self.config.cache_dir, self.config.use_cache)
        self.provider = TTSProvider_ElevenLabs(self.config)

        # ROS 통신 설정
        # 1) 사람이 "이 문장 말해"라고 보낼 곳
        self.sub_tts = self.create_subscription(
            String,
            "/tts",
            self._on_tts_text,
            10
        )

        # 2) 로봇 드라이버(go2_driver_node)가 구독할 곳
        #    go2_driver_node 쪽에서 self.create_subscription(String, "tts_audio_stream", ...) 했으니까
        #    여기서도 topic 이름 똑같이 "tts_audio_stream" 으로 맞춘다 (슬래시 없이!)
        self.pub_audio = self.create_publisher(
            String,
            "tts_audio_stream",
            10
        )

        self.get_logger().info("🎤 EnhancedTTSNode ready (pipeline: /tts -> /tts_audio_stream)")

    def _on_tts_text(self, msg: String) -> None:
        """
        /tts 에 문자열 오면 실행되는 콜백
        1) 캐시 확인 or 새로 합성
        2) 로컬재생 or 로봇으로 publish
        """
        try:
            text = msg.data.strip()
            if not text:
                self.get_logger().warn("TTS request was empty")
                return

            self.get_logger().info(f'🎤 TTS request: "{text}"')

            # 1) 캐시 확인
            mp3_bytes = self.cache.get(text, self.config.voice_name, "elevenlabs") \
                if self.config.use_cache else None

            if mp3_bytes:
                self.get_logger().info("💾 cache hit")
            else:
                self.get_logger().info("🔊 synthesizing with ElevenLabs...")
                mp3_bytes = self.provider.synthesize(text)
                if not mp3_bytes:
                    self.get_logger().error("❌ TTS synth failed")
                    return
                # 캐시에 저장
                self.cache.put(text, self.config.voice_name, "elevenlabs", mp3_bytes)

            # 길이 추정
            duration_sec = AudioInspector.estimate_duration_sec_from_mp3(mp3_bytes)

            # 2) local_playback이면 그냥 이 머신에서 스피커로 재생
            if self.config.local_playback:
                AudioInspector.local_play(mp3_bytes)
                self.get_logger().info("🔊 played locally")
            else:
                # 3) 아니면 로봇으로 보낼 패킷을 publish
                self._publish_to_robot(mp3_bytes, duration_sec)

            self.get_logger().info(f"✅ done (len={duration_sec:.2f}s)")
        except Exception as e:
            self.get_logger().error(f"❌ error in _on_tts_text: {e}")

    def _publish_to_robot(self, mp3_bytes: bytes, duration_sec: float) -> None:
        """
        로봇 드라이버(go2_driver_node)가 구독하는 tts_audio_stream 토픽으로
        {duration, mp3_b64} JSON을 보낸다.
        """
        try:
            payload = {
                "duration": float(duration_sec),
                "mp3_b64": base64.b64encode(mp3_bytes).decode("utf-8")
            }

            out_msg = String()
            out_msg.data = json.dumps(payload)

            self.pub_audio.publish(out_msg)

            self.get_logger().info(
                f"📤 published tts_audio_stream "
                f"(size={len(mp3_bytes)} bytes, dur={duration_sec:.2f}s)"
            )
        except Exception as e:
            self.get_logger().error(f"❌ failed to publish tts_audio_stream: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EnhancedTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
