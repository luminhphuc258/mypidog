#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import subprocess
import time
from time import sleep
from pathlib import Path
from typing import Dict, Optional, Union

from pidog import Pidog
from robot_hat import Servo


class MatthewPidogBootClass:
    """
    Mục tiêu: tránh robot té khi init Pidog trong lúc robot đang ngồi.

    Flow an toàn:
    (0) (optional) cleanup GPIO busy
    (1) unlock speaker (SPK_EN)
    (2) PREPOSE bằng robot_hat.Servo -> đưa robot về init-pose theo nhóm (êm + delay)
    (3) Init Pidog với đúng init angles (robot đã đứng sẵn -> không giật)
    (4) (optional) force head yaw bằng robot_hat
    """

    # ===== pins mapping (PCA9685 channels) =====
    LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
    HEAD_PINS = [8, 9, 10]   # [NECK_PITCH, NECK_TILT, HEAD_YAW]
    TAIL_PIN  = [11]

    # ===== init angles cố định (đúng như bạn yêu cầu) =====
    LEG_INIT_ANGLES  = [-3, 89, 9, -80, 3, 90, 10, -90]
    HEAD_INIT_ANGLES = [20, -45, -90]
    TAIL_INIT_ANGLE  = [30]   # MUST be list

    CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]

    def __init__(
        self,
        speaker_device: str = "plughw:3,0",
        cleanup_gpio: bool = True,
        kill_python: bool = False,

        # prepose trước khi init pidog
        enable_prepose: bool = True,
        prepose_step_delay: float = 0.05,   # tăng lên để êm hơn (0.04~0.08)
        prepose_settle_sec: float = 1.0,    # đứng yên sau khi prepose

        # force head yaw sau init
        enable_force_head: bool = True,
        force_head_port: str = "P10",
        force_head_angle: float = -90,
        force_hold: float = 0.25,
    ):
        self.speaker_device = speaker_device
        self.cleanup_gpio = cleanup_gpio
        self.kill_python = kill_python

        self.enable_prepose = enable_prepose
        self.prepose_step_delay = prepose_step_delay
        self.prepose_settle_sec = prepose_settle_sec

        self.enable_force_head = enable_force_head
        self.force_head_port = force_head_port
        self.force_head_angle = force_head_angle
        self.force_hold = force_hold

        self.dog: Optional[Pidog] = None

    # ===================== helpers =====================

    @staticmethod
    def _clamp(a: float) -> float:
        if a < -90:
            return -90
        if a > 90:
            return 90
        return a

    def _servo_set(self, port: str, angle: float):
        angle = self._clamp(float(angle))
        Servo(port).angle(angle)
        if self.prepose_step_delay and self.prepose_step_delay > 0:
            sleep(self.prepose_step_delay)

    # ===================== GPIO busy cleanup =====================

    def cleanup_gpio_busy(self):
        print("[CLEAN] Free GPIO devices...")
        subprocess.run(
            ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
            check=False,
        )
        if self.kill_python:
            subprocess.run(["bash", "-lc", "sudo killall -q python3 python || true"], check=False)
        time.sleep(0.25)

    # ===================== SPK_EN unlock =====================

    def _read_boot_config(self) -> str:
        for p in self.CONFIG_PATHS:
            if os.path.exists(p):
                try:
                    return open(p, "r", errors="ignore").read()
                except Exception:
                    pass
        return ""

    def detect_spk_en_pin(self) -> int:
        txt = self._read_boot_config()
        lines = [ln.split("#", 1)[0].strip() for ln in txt.splitlines()]
        overlays = [ln for ln in lines if ln.startswith("dtoverlay=")]

        if any("googlevoicehat-soundcard" in ln for ln in overlays):
            return 12
        if any("hifiberry-dac" in ln for ln in overlays):
            return 20

        # fallback: check soundcards
        try:
            out = subprocess.run(["aplay", "-l"], capture_output=True, text=True).stdout.lower()
            if "googlevoi" in out or "googlevoicehat" in out:
                return 12
        except Exception:
            pass

        return 20

    def set_gpio_high(self, pin: int) -> bool:
        if shutil.which("pinctrl"):
            subprocess.run(["pinctrl", "set", str(pin), "op", "dh"], check=False)
            return True
        if shutil.which("raspi-gpio"):
            subprocess.run(["raspi-gpio", "set", str(pin), "op", "dh"], check=False)
            return True
        return False

    def prime_speaker(self):
        # play 0.1s silence to "prime" amp
        import wave
        silence = "/tmp/robothat_silence.wav"
        if not os.path.exists(silence):
            with wave.open(silence, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b"\x00\x00" * (16000 // 10))
        subprocess.run(["aplay", "-D", self.speaker_device, "-q", silence], check=False)

    def unlock_robothat_speaker(self):
        pin = self.detect_spk_en_pin()
        if not self.set_gpio_high(pin):
            print("[WARN] Không tìm thấy pinctrl/raspi-gpio để set SPK_EN.")
            return False

        self.prime_speaker()
        subprocess.run(["amixer", "sset", "robot-hat speaker", "100%"], check=False)
        subprocess.run(["amixer", "sset", "robot-hat speaker Playback Volume", "100%"], check=False)
        print(f"[OK] Speaker unlocked (SPK_EN GPIO{pin})")
        return True

    # ===================== PREPOSE (robot_hat only) =====================

    def _build_init_pose_dict(self) -> Dict[str, float]:
        pose: Dict[str, float] = {}
        # legs P0..P7
        for i, ang in enumerate(self.LEG_INIT_ANGLES):
            pose[f"P{i}"] = float(ang)
        # head P8..P10
        pose["P8"] = float(self.HEAD_INIT_ANGLES[0])
        pose["P9"] = float(self.HEAD_INIT_ANGLES[1])
        pose["P10"] = float(self.HEAD_INIT_ANGLES[2])
        # tail P11
        pose["P11"] = float(self.TAIL_INIT_ANGLE[0])
        return pose

    def prepose_to_init_gentle(self):
        """
        Đưa robot từ tư thế đang ngồi -> về init pose theo nhóm để tránh té.
        Không dùng pidog.
        """
        pose = self._build_init_pose_dict()

        print("[PREPOSE] Move to INIT pose gently (robot_hat only)")

        # Nhóm move (êm + chống té):
        # 1) head + tail trước (đỡ lệch trọng tâm)
        group_head = ["P8", "P9", "P10", "P11"]

        # 2) rear-left (P4,P5) rồi chờ 1s
        group_rear_left = ["P4", "P5"]

        # 3) rear-right (P6,P7) rồi chờ 1s
        group_rear_right = ["P6", "P7"]

        # 4) front legs cuối cùng (P0..P3)
        group_front = ["P0", "P1", "P2", "P3"]

        for port in group_head:
            self._servo_set(port, pose[port])
        sleep(0.25)

        for port in group_rear_left:
            self._servo_set(port, pose[port])
        print("[PREPOSE] settle rear-left 1.0s")
        sleep(1.0)

        for port in group_rear_right:
            self._servo_set(port, pose[port])
        print("[PREPOSE] settle rear-right 1.0s")
        sleep(1.0)

        for port in group_front:
            self._servo_set(port, pose[port])

        if self.prepose_settle_sec and self.prepose_settle_sec > 0:
            print(f"[PREPOSE] final settle {self.prepose_settle_sec:.1f}s")
            sleep(self.prepose_settle_sec)

        return True

    # ===================== FORCE HEAD (robot_hat only) =====================

    def force_head(self):
        try:
            Servo(self.force_head_port).angle(self._clamp(self.force_head_angle))
            sleep(self.force_hold)
            print(f"[FORCE] {self.force_head_port} -> {self.force_head_angle}°")
            return True
        except Exception as e:
            print(f"[FORCE WARN] {e}")
            return False

    # ===================== CREATE PIDOG =====================

    def create(self) -> Pidog:
        """
        Create dog object (Pidog) an toàn: prepose trước để tránh giật té.
        """
        if self.cleanup_gpio:
            self.cleanup_gpio_busy()

        self.unlock_robothat_speaker()

        if self.enable_prepose:
            # prepose êm trước khi pidog init
            self.prepose_to_init_gentle()

        # BÂY GIỜ robot đã gần/đúng init pose -> gọi Pidog sẽ không giật mạnh nữa
        print("[INIT] Creating Pidog() ...")
        self.dog = Pidog(
            leg_pins=self.LEG_PINS,
            head_pins=self.HEAD_PINS,
            tail_pin=self.TAIL_PIN,
            leg_init_angles=self.LEG_INIT_ANGLES,
            head_init_angles=self.HEAD_INIT_ANGLES,
            tail_init_angle=self.TAIL_INIT_ANGLE
        )

        # chờ servo settle
        if hasattr(self.dog, "wait_all_done"):
            self.dog.wait_all_done()
        sleep(0.4)

        if self.enable_force_head:
            self.force_head()

        print("[INIT] Pidog ready.")
        return self.dog

    def close(self):
        try:
            if self.dog:
                self.dog.close()
        except Exception:
            pass
