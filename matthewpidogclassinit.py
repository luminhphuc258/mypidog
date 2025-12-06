#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import subprocess
import time
from pathlib import Path
from time import sleep

from pidog import Pidog
from robot_hat import Servo


class MatthewPidogBootClass:
    """
    Boot PiDog safely:
    (A) optional cleanup GPIO busy
    (B) unlock speaker via SPK_EN
    (C) optional prepose using robot_hat.Servo from a pose file (NOT pidog)
    (D) init Pidog with fixed init arrays
    (E) optional force head servo after init (bypass pidog limits)
    """

    LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
    HEAD_PINS = [8, 9, 10]
    TAIL_PIN  = [11]

    # fixed arrays (as you want)
    LEG_INIT_ANGLES  = [-3, 89, 9, -80, 3, 90, 10, -90]
    HEAD_INIT_ANGLES = [20, -45, -90]
    TAIL_INIT_ANGLE  = [30]  # must be list

    CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]

    def __init__(
        self,
        speaker_device: str = "plughw:3,0",
        pose_file: str | Path = "pidog_pose_config.txt",

        enable_prepose: bool = False,     # default OFF (ổn định hơn)
        prepose_delay_sec: float = 1.0,
        prepose_step_delay: float = 0.02,

        enable_force_head: bool = True,
        force_head_port: str = "P10",
        force_head_angle: float = -90,

        cleanup_gpio: bool = False,
        kill_python: bool = False,
    ):
        self.speaker_device = speaker_device
        self.pose_file = Path(pose_file) if not isinstance(pose_file, Path) else pose_file

        self.enable_prepose = enable_prepose
        self.prepose_delay_sec = float(prepose_delay_sec)
        self.prepose_step_delay = float(prepose_step_delay)

        self.enable_force_head = enable_force_head
        self.force_head_port = force_head_port
        self.force_head_angle = float(force_head_angle)

        self.cleanup_gpio = cleanup_gpio
        self.kill_python = kill_python

        self.dog: Pidog | None = None

    # -------------------- helpers --------------------

    @staticmethod
    def _clamp(a: float) -> float:
        if a < -90:
            return -90
        if a > 90:
            return 90
        return a

    # -------------------- GPIO busy cleanup --------------------

    def cleanup_gpio_busy(self):
        print("[CLEAN] Free GPIO devices...")
        subprocess.run(
            ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
            check=False
        )
        if self.kill_python:
            subprocess.run(["bash", "-lc", "sudo killall -q python3 python || true"], check=False)
        time.sleep(0.2)

    # -------------------- speaker unlock --------------------

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

        # fallback: detect from aplay -l
        try:
            out = subprocess.run(["aplay", "-l"], capture_output=True, text=True).stdout.lower()
            if "googlevoi" in out or "googlevoicehat" in out:
                return 12
        except Exception:
            pass

        return 20

    @staticmethod
    def _set_gpio_high(pin: int) -> bool:
        if shutil.which("pinctrl"):
            subprocess.run(["pinctrl", "set", str(pin), "op", "dh"], check=False)
            return True
        if shutil.which("raspi-gpio"):
            subprocess.run(["raspi-gpio", "set", str(pin), "op", "dh"], check=False)
            return True
        return False

    def _prime_speaker(self):
        # play short silence to "prime" amp
        import wave
        silence = "/tmp/robothat_silence.wav"
        if not os.path.exists(silence):
            with wave.open(silence, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b"\x00\x00" * (16000 // 10))
        subprocess.run(["aplay", "-D", self.speaker_device, "-q", silence], check=False)

    def unlock_robothat_speaker(self) -> bool:
        pin = self.detect_spk_en_pin()
        ok = self._set_gpio_high(pin)
        if not ok:
            print("[WARN] No pinctrl/raspi-gpio (or no permission).")
            return False

        self._prime_speaker()
        subprocess.run(["amixer", "sset", "robot-hat speaker", "100%"], check=False)
        subprocess.run(["amixer", "sset", "robot-hat speaker Playback Volume", "100%"], check=False)

        print(f"[OK] Speaker unlocked (SPK_EN GPIO{pin})")
        return True

    # -------------------- optional prepose from file --------------------

    def _parse_pose_file(self) -> dict[int, float]:
        """
        Parse pose in either JSON or text lines.
        Returns {channel:int -> angle:float} for P0..P11.
        """
        if not self.pose_file.exists():
            return {}

        txt = self.pose_file.read_text(encoding="utf-8", errors="ignore").strip()
        if not txt:
            return {}

        # JSON
        import json, re
        try:
            obj = json.loads(txt)
            out: dict[int, float] = {}
            for k, v in obj.items():
                m = re.search(r"(\d+)", str(k))
                if m:
                    out[int(m.group(1))] = float(v)
            return out
        except Exception:
            pass

        # Text
        import re
        out: dict[int, float] = {}
        for line in txt.splitlines():
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue
            m = re.search(r"[Pp]\s*(\d+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
            if m:
                out[int(m.group(1))] = float(m.group(2))
        return out

    def apply_pose_from_file(self) -> bool:
        pose = self._parse_pose_file()
        if not pose:
            print(f"[WARN] pose file empty/invalid: {self.pose_file} (skip prepose)")
            return False

        order = list(range(0, 8)) + [8, 9, 10] + [11]
        print(f"[PREPOSE] Applying pose: {self.pose_file}")

        for ch in order:
            if ch not in pose:
                continue
            port = f"P{ch}"
            ang = self._clamp(float(pose[ch]))
            try:
                Servo(port).angle(ang)
                if self.prepose_step_delay > 0:
                    sleep(self.prepose_step_delay)
            except Exception as e:
                print(f"[PREPOSE WARN] {port} -> {ang} fail: {e}")

        if self.prepose_delay_sec > 0:
            print(f"[PREPOSE] Stabilize {self.prepose_delay_sec:.1f}s ...")
            time.sleep(self.prepose_delay_sec)

        return True

    # -------------------- force servo --------------------

    def force_servo_angle(self, port: str, angle: float, hold: float = 0.3) -> bool:
        angle = self._clamp(float(angle))
        try:
            Servo(port).angle(angle)
            sleep(hold)
            print(f"[FORCE] {port} -> {angle} deg (bypass pidog)")
            return True
        except Exception as e:
            print(f"[FORCE ERROR] {port}: {e}")
            return False

    # -------------------- create pidog --------------------

    def create(self) -> Pidog:
        if self.cleanup_gpio:
            self.cleanup_gpio_busy()

        self.unlock_robothat_speaker()

        if self.enable_prepose:
            self.apply_pose_from_file()

        print("[BOOT] Init Pidog...")
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

        if self.enable_force_head:
            sleep(0.2)
            self.force_servo_angle(self.force_head_port, self.force_head_angle, hold=0.4)

        return self.dog
