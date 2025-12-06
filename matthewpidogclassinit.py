#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import json
import shutil
import subprocess
import time
from pathlib import Path
from time import sleep

from pidog import Pidog
from robot_hat import Servo


class MatthewPidogBootClass:
    """
    Flow mới:
    (A) (Optional) cleanup GPIO busy
    (B) Unlock speaker (SPK_EN)
    (C) Pre-pose: đọc pose file -> set servo bằng robot_hat.Servo (KHÔNG dùng pidog)
        -> delay 1s cho ổn định
    (D) Init Pidog với mảng góc cố định như hiện tại
    (E) (Optional) Force head servo sau init (bypass pidog)
    """

    # ===== fixed pins mapping =====
    LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
    HEAD_PINS = [8, 9, 10]   # [NECK_PITCH, NECK_TILT, HEAD_YAW]
    TAIL_PIN  = [11]

    # ✅ init angles cố định (như bạn yêu cầu giữ nguyên)
    LEG_INIT_ANGLES  = [-3, 89, 9, -80, 3, 90, 10, -90]
    HEAD_INIT_ANGLES = [20, -45, -90]
    TAIL_INIT_ANGLE  = [30]  # MUST be list

    CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]

    def __init__(
        self,
        speaker_device: str = "plughw:3,0",
        pose_file: str | Path = "pidog_pose_config.txt",

        # pre-pose (robot_hat only, before pidog init)
        enable_prepose: bool = True,
        prepose_delay_sec: float = 1.0,
        prepose_step_delay: float = 0.02,  # delay nhẹ giữa các servo để đỡ giật

        # after pidog init
        enable_force_head: bool = True,
        force_head_port: str = "P10",
        force_head_angle: float = -90,

        # GPIO busy handling
        cleanup_gpio: bool = False,
        kill_python: bool = False,
    ):
        self.speaker_device = speaker_device
        self.pose_file = Path(pose_file) if not isinstance(pose_file, Path) else pose_file

        self.enable_prepose = enable_prepose
        self.prepose_delay_sec = prepose_delay_sec
        self.prepose_step_delay = prepose_step_delay

        self.enable_force_head = enable_force_head
        self.force_head_port = force_head_port
        self.force_head_angle = force_head_angle

        self.cleanup_gpio = cleanup_gpio
        self.kill_python = kill_python

        self.dog = None

    # ===================== GPIO CLEANUP (optional) =====================

    def cleanup_gpio_busy(self):
        print("[CLEAN] Free GPIO devices...")
        subprocess.run(
            ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
            check=False
        )
        if self.kill_python:
            subprocess.run(["bash", "-lc", "sudo killall -q python3 python || true"], check=False)
        time.sleep(0.2)

    # ===================== AUDIO UNLOCK (SPK_EN) =====================

    def _read_boot_config(self):
        for p in self.CONFIG_PATHS:
            if os.path.exists(p):
                try:
                    return open(p, "r", errors="ignore").read()
                except:
                    pass
        return ""

    def detect_spk_en_pin(self):
        txt = self._read_boot_config()
        lines = [ln.split("#", 1)[0].strip() for ln in txt.splitlines()]
        overlays = [ln for ln in lines if ln.startswith("dtoverlay=")]

        if any("googlevoicehat-soundcard" in ln for ln in overlays):
            return 12
        if any("hifiberry-dac" in ln for ln in overlays):
            return 20

        try:
            out = subprocess.run(["aplay", "-l"], capture_output=True, text=True).stdout.lower()
            if "googlevoi" in out or "googlevoicehat" in out:
                return 12
        except:
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
        ok = self.set_gpio_high(pin)
        if not ok:
            print("[WARN] Không tìm thấy pinctrl/raspi-gpio (hoặc thiếu quyền).")
            return False

        self.prime_speaker()
        subprocess.run(["amixer", "sset", "robot-hat speaker", "100%"], check=False)
        subprocess.run(["amixer", "sset", "robot-hat speaker Playback Volume", "100%"], check=False)

        print(f"[OK] Speaker unlocked (SPK_EN GPIO{pin})")
        return True

    # ===================== PRE-POSE (robot_hat only) =====================

    def _parse_pose_file(self, path: Path) -> dict[int, float]:
        """
        Trả dict {channel:int -> angle:float} cho P0..P11
        Hỗ trợ JSON hoặc text kiểu:
          P0: -3
          P1 = 89
          ...
        """
        txt = path.read_text(encoding="utf-8", errors="ignore").strip()

        # JSON first
        try:
            obj = json.loads(txt)
            out = {}
            for k, v in obj.items():
                m = re.search(r"(\d+)", str(k))
                if m:
                    out[int(m.group(1))] = float(v)
            return out
        except Exception:
            pass

        out = {}
        for line in txt.splitlines():
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue

            m = re.search(r"[Pp]\s*(\d+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
            if m:
                out[int(m.group(1))] = float(m.group(2))
                continue

            nums = re.findall(r"-?\d+(?:\.\d+)?", line)
            if len(nums) >= 2:
                ch = int(float(nums[0]))
                ang = float(nums[1])
                out[ch] = ang

        return out

    def _clamp(self, a: float) -> float:
        if a < -90: return -90
        if a > 90:  return 90
        return a

    def apply_pose_from_file(self):
        """
        Set servo theo pose file bằng robot_hat.Servo, không gọi pidog.
        Set theo thứ tự: legs -> head -> tail để ít té.
        """
        if not self.pose_file.exists():
            print(f"[WARN] Pose file not found: {self.pose_file} (skip prepose)")
            return False

        pose = self._parse_pose_file(self.pose_file)
        if not pose:
            print(f"[WARN] Pose file parse fail/empty: {self.pose_file} (skip prepose)")
            return False

        # build angles list with available ones
        order = list(range(0, 8)) + [8, 9, 10] + [11]  # legs, head, tail
        print(f"[PREPOSE] Applying pose from: {self.pose_file}")

        for ch in order:
            if ch not in pose:
                continue
            ang = self._clamp(float(pose[ch]))
            port = f"P{ch}"
            try:
                Servo(port).angle(ang)
                if self.prepose_step_delay and self.prepose_step_delay > 0:
                    sleep(self.prepose_step_delay)
            except Exception as e:
                print(f"[PREPOSE WARN] {port} -> {ang} failed: {e}")

        if self.prepose_delay_sec and self.prepose_delay_sec > 0:
            print(f"[PREPOSE] Stabilize {self.prepose_delay_sec:.1f}s ...")
            time.sleep(self.prepose_delay_sec)

        return True

    # ===================== FORCE SERVO =====================

    def force_servo_angle(self, port: str, angle: float, hold=0.3):
        angle = self._clamp(float(angle))
        try:
            Servo(port).angle(angle)
            sleep(hold)
            print(f"[FORCE] {port} -> {angle} deg (bypass pidog)")
            return True
        except Exception as e:
            print(f"[FORCE ERROR] {port}: {e}")
            return False

    # ===================== CREATE PIDOG =====================

    def create(self) -> Pidog:
        # (A) cleanup GPIO busy trước (nếu cần)
        if self.cleanup_gpio:
            self.cleanup_gpio_busy()

        # (B) unlock speaker
        self.unlock_robothat_speaker()

        # (C) pre-pose bằng robot_hat (nếu bật)
        if self.enable_prepose:
            self.apply_pose_from_file()

        # (D) init pidog với mảng cố định
        print("\nInit PiDog (fixed init arrays)...")
        print("LEG_PINS :", self.LEG_PINS,  "angles:", self.LEG_INIT_ANGLES)
        print("HEAD_PINS:", self.HEAD_PINS, "angles:", self.HEAD_INIT_ANGLES)
        print("TAIL_PIN :", self.TAIL_PIN,  "angle :", self.TAIL_INIT_ANGLE)

        self.dog = Pidog(
            leg_pins=self.LEG_PINS,
            head_pins=self.HEAD_PINS,
            tail_pin=self.TAIL_PIN,
            leg_init_angles=self.LEG_INIT_ANGLES,
            head_init_angles=self.HEAD_INIT_ANGLES,
            tail_init_angle=self.TAIL_INIT_ANGLE
        )

        if hasattr(self.dog, "wait_all_done"):
            self.dog.wait_all_done()

        # (E) force head sau init
        if self.enable_force_head:
            sleep(0.2)
            self.force_servo_angle(self.force_head_port, self.force_head_angle, hold=0.4)

        return self.dog


