#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import subprocess
import time
from time import sleep

from pidog import Pidog
from robot_hat import Servo


class MatthewPidogBootClass:
    """
    - Unlock Robot-HAT/PiDog speaker via SPK_EN GPIO (như i2samp.sh)
    - Init Pidog theo mảng góc LEG_INIT_ANGLES cố định (KHÔNG đọc file config)
    - (Optional) Force head servo sau init (bypass giới hạn pidog)
    """

    # ===== fixed pins mapping =====
    LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
    HEAD_PINS = [8, 9, 10]   # [NECK_PITCH, NECK_TILT, HEAD_YAW]
    TAIL_PIN  = [11]

    # ✅ fixed init angles (theo bạn yêu cầu)
    LEG_INIT_ANGLES = [-3, 60, 9, -60, 3, 60, 10, -60]

    # giữ nguyên như trước
    HEAD_INIT_ANGLES = [20, -45, -90]
    TAIL_INIT_ANGLE  = [30]  # MUST be list

    CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]

    def __init__(
        self,
        speaker_device: str = "plughw:3,0",
        enable_force_head: bool = True,
        force_head_port: str = "P10",
        force_head_angle: float = -90,
        stabilize_sec: float = 1.0,   # cho servo ổn định chống té
        cleanup_gpio: bool = False,   # nếu muốn auto dọn GPIO busy
        kill_python: bool = False,    # mạnh tay (chỉ dùng khi bị GPIO busy hoài)
    ):
        self.speaker_device = speaker_device
        self.enable_force_head = enable_force_head
        self.force_head_port = force_head_port
        self.force_head_angle = force_head_angle
        self.stabilize_sec = stabilize_sec

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

    # ===================== FORCE SERVO =====================

    def force_servo_angle(self, port: str, angle: float, hold=0.3):
        if angle < -90: angle = -90
        if angle > 90:  angle = 90
        try:
            s = Servo(port)
            s.angle(angle)
            sleep(hold)
            print(f"[FORCE] {port} -> {angle}° (bypass pidog)")
            return True
        except Exception as e:
            print(f"[FORCE ERROR] {port}: {e}")
            return False

    # ===================== CREATE PIDOG =====================

    def create(self) -> Pidog:
        if self.cleanup_gpio:
            self.cleanup_gpio_busy()

        self.unlock_robothat_speaker()

        print("\nInit PiDog (fixed LEG_INIT_ANGLES)...")
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

        # force head after init
        if self.enable_force_head:
            sleep(0.2)
            self.force_servo_angle(self.force_head_port, self.force_head_angle, hold=0.4)

        # stabilize
        if self.stabilize_sec and self.stabilize_sec > 0:
            print(f"[STABLE] waiting {self.stabilize_sec:.1f}s for servos to stabilize...")
            time.sleep(self.stabilize_sec)

        return self.dog
