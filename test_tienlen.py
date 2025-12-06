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


# ===================== USER CONFIG =====================

SPEAKER_DEVICE = "plughw:3,0"
POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"

LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
HEAD_PINS = [8, 9, 10]   # [NECK_PITCH, NECK_TILT, HEAD_YAW]
TAIL_PIN  = [11]

# giữ nguyên theo bạn
HEAD_INIT_ANGLES = [20, -45, -90]
TAIL_INIT_ANGLE  = [30]  # MUST be list

FORCE_HEAD_PORT  = "P10"
FORCE_HEAD_ANGLE = -90

CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]


# ===================== GPIO CLEANUP (FIX GPIO BUSY) =====================

def cleanup_gpio_busy(kill_python=False):
    """
    Kill tiến trình đang giữ /dev/gpiochip* /dev/gpiomem /dev/mem
    -> fix lgpio/gpiozero: GPIO busy
    """
    print("[CLEAN] Free GPIO devices...")
    subprocess.run(
        ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
        check=False
    )

    # optional: mạnh tay, chỉ bật khi bạn muốn kill hết python cũ
    if kill_python:
        subprocess.run(["bash", "-lc", "sudo killall -q python3 python || true"], check=False)

    # nghỉ chút cho kernel release
    time.sleep(0.2)


# ===================== POSE LOADER (P0..P7) =====================

def load_channels_from_pose_file(path: Path) -> dict[int, float]:
    """
    Đọc pose file, trả về dict {channel:int -> angle:float}
    Hỗ trợ:
      - JSON: {"P0": -3, "P1": 89, ...} hoặc {"0": -3, ...}
      - Text: P0: -3 / P1 = 89 / P2 -10 ...
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

    # Text parse
    out = {}
    for line in txt.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue

        # P0: -3 or P1 = 89
        m = re.search(r"[Pp]\s*(\d+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            out[int(m.group(1))] = float(m.group(2))
            continue

        # fallback "0 -3"
        nums = re.findall(r"-?\d+(?:\.\d+)?", line)
        if len(nums) >= 2:
            ch = int(float(nums[0]))
            ang = float(nums[1])
            out[ch] = ang

    return out


def load_leg_init_angles_or_fallback(fallback):
    """
    Chỉ load P0..P7 để thay LEG_INIT_ANGLES.
    """
    if not POSE_FILE.exists():
        print(f"[WARN] Pose file not found: {POSE_FILE} -> use fallback legs.")
        return fallback

    pose = load_channels_from_pose_file(POSE_FILE)
    missing = [i for i in range(8) if i not in pose]
    if missing:
        print(f"[WARN] Pose file thiếu {missing} (P0..P7) -> use fallback legs.")
        return fallback

    angles = [pose[i] for i in range(8)]
    print(f"[OK] LEG_INIT_ANGLES from file: {angles}")
    return angles


# ===================== AUDIO UNLOCK (SPK_EN) =====================

def _read_boot_config():
    for p in CONFIG_PATHS:
        if os.path.exists(p):
            try:
                return open(p, "r", errors="ignore").read()
            except:
                pass
    return ""


def detect_spk_en_pin():
    txt = _read_boot_config()
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


def set_gpio_high(pin: int) -> bool:
    if shutil.which("pinctrl"):
        subprocess.run(["pinctrl", "set", str(pin), "op", "dh"], check=False)
        return True
    if shutil.which("raspi-gpio"):
        subprocess.run(["raspi-gpio", "set", str(pin), "op", "dh"], check=False)
        return True
    return False


def prime_speaker(device: str):
    import wave
    silence = "/tmp/robothat_silence.wav"
    if not os.path.exists(silence):
        with wave.open(silence, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(b"\x00\x00" * (16000 // 10))
    subprocess.run(["aplay", "-D", device, "-q", silence], check=False)


def unlock_robothat_speaker(device: str):
    pin = detect_spk_en_pin()
    ok = set_gpio_high(pin)
    if not ok:
        print("[WARN] Không tìm thấy pinctrl/raspi-gpio (hoặc thiếu quyền).")
        return False

    prime_speaker(device)
    subprocess.run(["amixer", "sset", "robot-hat speaker", "100%"], check=False)
    subprocess.run(["amixer", "sset", "robot-hat speaker Playback Volume", "100%"], check=False)

    print(f"[OK] Speaker unlocked (SPK_EN GPIO{pin})")
    return True


# ===================== FORCE SERVO =====================

def force_servo_angle(port: str, angle: float, hold=0.3):
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


# ===================== MAIN DEMO =====================

def main():
    # 0) free GPIO (fix GPIO busy)
    cleanup_gpio_busy(kill_python=False)

    # 1) unlock speaker
    unlock_robothat_speaker(SPEAKER_DEVICE)

    # 2) load legs from file to replace fixed array
    LEG_INIT_ANGLES_FALLBACK = [-3, 89, 9, -80, 3, 90, 10, -90]
    leg_init_angles = load_leg_init_angles_or_fallback(LEG_INIT_ANGLES_FALLBACK)

    print("\nInit PiDog...")
    print("LEG_PINS :", LEG_PINS,  "angles:", leg_init_angles)
    print("HEAD_PINS:", HEAD_PINS, "angles:", HEAD_INIT_ANGLES)
    print("TAIL_PIN :", TAIL_PIN,  "angle :", TAIL_INIT_ANGLE)

    my_dog = Pidog(
        leg_pins=LEG_PINS,
        head_pins=HEAD_PINS,
        tail_pin=TAIL_PIN,
        leg_init_angles=leg_init_angles,
        head_init_angles=HEAD_INIT_ANGLES,
        tail_init_angle=TAIL_INIT_ANGLE
    )

    if hasattr(my_dog, "wait_all_done"):
        my_dog.wait_all_done()

    # 3) force head after init
    sleep(0.2)
    force_servo_angle(FORCE_HEAD_PORT, FORCE_HEAD_ANGLE, hold=0.4)

    # 4) stabilize 1s to avoid falling
    print("[STABLE] waiting 1.0s for servos to stabilize...")
    time.sleep(1.0)

    # 5) demo fast walk + fast turns + sit
    try:
        my_dog.rgb_strip.set_mode('breath', 'white', bps=0.8)
        my_dog.do_action('stand', speed=95)
        my_dog.wait_all_done()
        time.sleep(0.2)

        my_dog.rgb_strip.set_mode('breath', 'white', bps=0.7)
        my_dog.do_action('forward', step_count=6, speed=99)
        my_dog.wait_all_done()
        time.sleep(0.1)

        my_dog.rgb_strip.set_mode('boom', 'yellow', bps=3)
        my_dog.do_action('turn_left', step_count=5, speed=99)
        my_dog.wait_all_done()
        time.sleep(0.05)

        my_dog.rgb_strip.set_mode('boom', 'blue', bps=3)
        my_dog.do_action('turn_right', step_count=5, speed=99)
        my_dog.wait_all_done()
        time.sleep(0.05)

        my_dog.rgb_strip.set_mode('breath', 'red', bps=0.8)
        my_dog.do_action('sit', speed=90)
        my_dog.wait_all_done()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        try:
            my_dog.close()
        except:
            pass


if __name__ == "__main__":
    main()
