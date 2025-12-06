#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- Unlock Robot-HAT/PiDog speaker via SPK_EN GPIO (như i2samp.sh)
- Init Pidog theo pose bạn set
- Chỉ thay LEG_INIT_ANGLES bằng giá trị lấy từ pidog_pose_config.txt (P0..P7)
- Các phần còn lại giữ nguyên
- Sau init: ép xoay head servo (bypass giới hạn pidog) về -90 bằng robot_hat.Servo
"""

import os
import re
import json
import shutil
import subprocess
from time import sleep
from pathlib import Path

from pidog import Pidog
from robot_hat import Servo

# ===================== LOAD LEG POSE FROM FILE =====================

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"

def load_channels_from_pose_file(path: Path) -> dict[int, float]:
    """
    Đọc file pose, trả về dict {channel:int -> angle:float}
    Hỗ trợ format:
      - JSON: {"P0": -3, "P1": 89, ...} hoặc {"0": -3, ...}
      - Text: P0: -3 / P1 = 89 / P2 -10 ...
    """
    txt = path.read_text(encoding="utf-8", errors="ignore").strip()

    # Try JSON first
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

    # Parse text lines
    out = {}
    for line in txt.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue

        # match like "P0: -3" or "P1 = 89"
        m = re.search(r"[Pp]\s*(\d+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            out[int(m.group(1))] = float(m.group(2))
            continue

        # match like "0 -3"
        nums = re.findall(r"-?\d+(?:\.\d+)?", line)
        if len(nums) >= 2:
            ch = int(float(nums[0]))
            ang = float(nums[1])
            out[ch] = ang

    return out

def load_leg_init_angles_or_fallback(fallback_list):
    """
    Chỉ lấy P0..P7 từ pose file để thay LEG_INIT_ANGLES.
    Nếu file thiếu / parse fail -> dùng fallback_list.
    """
    try:
        if not POSE_FILE.exists():
            print(f"[WARN] Không thấy {POSE_FILE}, dùng LEG_INIT_ANGLES mặc định.")
            return fallback_list

        pose = load_channels_from_pose_file(POSE_FILE)
        missing = [i for i in range(8) if i not in pose]
        if missing:
            print(f"[WARN] Pose file thiếu {missing} (P0..P7). Dùng LEG_INIT_ANGLES mặc định.")
            return fallback_list

        angles = [pose[i] for i in range(8)]
        print(f"[OK] Loaded LEG_INIT_ANGLES from {POSE_FILE}: {angles}")
        return angles

    except Exception as e:
        print(f"[WARN] Lỗi đọc pose file: {e}. Dùng LEG_INIT_ANGLES mặc định.")
        return fallback_list


# ===================== AUDIO UNLOCK (SPK_EN) =====================

SPEAKER_DEVICE = "plughw:3,0"
CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]

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
            wf.writeframes(b"\x00\x00" * (16000 // 10))  # 0.1s
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


# ===================== PIDOG INIT POSE =====================

LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
HEAD_PINS = [8, 9, 10]   # [NECK_PITCH, NECK_TILT, HEAD_YAW]
TAIL_PIN  = [11]

# fallback (nếu file pose lỗi/mất)
LEG_INIT_ANGLES_FALLBACK = [-3, 89, 9, -80, 3, 90, 10, -90]

# >>> chỉ thay LEG_INIT_ANGLES bằng file config <<<
LEG_INIT_ANGLES = load_leg_init_angles_or_fallback(LEG_INIT_ANGLES_FALLBACK)

# giữ nguyên như bạn yêu cầu
HEAD_INIT_ANGLES = [20, -45, -90]
TAIL_INIT_ANGLE  = [30]   # MUST be list


# ===================== FORCE HEAD SERVO AFTER INIT =====================

FORCE_HEAD_PORT  = "P10"
FORCE_HEAD_ANGLE = -90

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
        print(f"[FORCE ERROR] không set được {port}: {e}")
        return False


def main():
    print("=== Unlock speaker + Init PiDog + Force head servo ===")
    unlock_robothat_speaker(SPEAKER_DEVICE)

    print("\nInit PiDog...")
    print("POSE_FILE:", POSE_FILE)
    print("LEG_PINS :", LEG_PINS,  "angles:", LEG_INIT_ANGLES)
    print("HEAD_PINS:", HEAD_PINS, "angles:", HEAD_INIT_ANGLES)
    print("TAIL_PIN :", TAIL_PIN,  "angle :", TAIL_INIT_ANGLE)

    dog = Pidog(
        leg_pins=LEG_PINS,
        head_pins=HEAD_PINS,
        tail_pin=TAIL_PIN,
        leg_init_angles=LEG_INIT_ANGLES,
        head_init_angles=HEAD_INIT_ANGLES,
        tail_init_angle=TAIL_INIT_ANGLE
    )

    if hasattr(dog, "wait_all_done"):
        dog.wait_all_done()

    sleep(0.2)
    force_servo_angle(FORCE_HEAD_PORT, FORCE_HEAD_ANGLE, hold=0.4)

    print("\n[DONE] Init xong + đã force head servo.")


if __name__ == "__main__":
    main()
