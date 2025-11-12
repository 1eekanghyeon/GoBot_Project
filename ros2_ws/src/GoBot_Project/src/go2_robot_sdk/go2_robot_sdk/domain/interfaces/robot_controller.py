# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from abc import ABC, abstractmethod


class IRobotController(ABC):
    """Interface for robot control operations"""

    @abstractmethod
    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        pass

    @abstractmethod
    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        pass

    @abstractmethod
    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        pass

    @abstractmethod
    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: str, topic: str) -> None:
        """Send WebRTC request"""
        pass 
    
    @abstractmethod
    def play_audio_track(self, robot_id: str, mp3_bytes: bytes, duration_sec: float) -> None:
        """
        Play an MP3 clip on the robot's speaker using the *existing*
        WebRTC connection.

        NOTE:
        - This MUST NOT open a new WebRTC session.
        - It MUST reuse the already connected peer (the same one used for movement / sensing).
        - mp3_bytes is the full compressed MP3 audio data (already TTS-generated).
        - duration_sec is how long the clip lasts, used to know how long to keep
          the outbound audio track alive.
        """
        pass