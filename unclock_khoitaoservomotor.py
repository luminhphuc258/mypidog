#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- Unlock Robot-HAT/PiDog speaker via SPK_EN GPIO (như i2samp.sh)
- Init Pidog theo pose lấy từ file pidog_pose_config.txt
- Sau init: ép xoay head servo (bypass pidog) về -90 bằng robot_hat.Servo
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


# ===================== POSE FILE =====================

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"

# Mapping pin bạn đã dùng (P0..P11)
LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
HEAD_PINS = [8, 9, 10]   # [NECK_PITCH, NECK_TILT, HEAD_YAW]
TAIL_PIN  = [11]

def load_pose_angles(path: Path) -> dict[int, float]:
    """
    Đọc pose file -> dict {0..11: angle}.
    Hỗ trợ:
      - JSON: {"P0": -3, "P1": 89, ...} hoặc {"0": -3, ...}
      - Text: P0: -3  /  P1 = 89  /  0 -3
    """
    txt = path.read_text(encoding="utf-8", errors="ignore").strip()

    # thử JSON trước
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

    # parse text
    out = {}
    for line in txt.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue

        # bắt kiểu "P0: -3" hoặc "P0 = -3"
        m = re.match(r"^[Pp]\s*(\d+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            out[int(m.group(1))] = float(m.group(2))
            continue

        # bắt kiểu "0 -3" hoặc "0, -3"
        nums = re.findall(r"-?\d+(?:\.\d+)?", line)
        if len(nums) >= 2:
            pin = int(float(nums[0]))
            ang = float(nums[1])
            out[pin] = ang

    return out

def build_init_arrays(pose: dict[int, float]):
    # thiếu key nào thì báo rõ
    missing = [p for p in (LEG_PINS + HEAD_PINS + TAIL_PIN) if p not in pose]
    if missing:
        raise ValueError(f"Pose file thiếu các kênh: {missing}")

    leg_init  = [pose[p] for p in LEG_PINS]
    head_init = [pose[p] for p in HEAD_PINS]
    tail_init = [pose[TAIL_PIN[0]]]   # tail MUST be list
    return leg_init, head_init, tail_init


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


# ===================== FORCE HEAD SERVO AFTER INIT =====================

# Bạn muốn ép “head index 3” về -90 (thường là HEAD_YAW = P10)
FORCE_HEAD_PORT  = "P10"
FORCE_HEAD_ANGLE = -90

def force_servo_angle(port: str, angle: float, hold=0.3):
    if angle < -90: angle = -90
    if angle > 90:  angle = 90
    try:
        s = Servo(port)
        s.angle(angle)
        sleep(hold)
        print(f"[FORCE] {port} -> {angle}°")
        return True
    except Exception as e:
        print(f"[FORCE ERROR] {port}: {e}")
        return False


# ===================== MAIN =====================

def main():
    print("=== Unlock speaker + Init PiDog from pose file ===")
    print(f"Speaker device: {SPEAKER_DEVICE}")
    unlock_robothat_speaker(SPEAKER_DEVICE)

    if not POSE_FILE.exists():
        raise FileNotFoundError(f"Không thấy file pose: {POSE_FILE}")

    pose = load_pose_angles(POSE_FILE)
    leg_init, head_init, tail_init = build_init_arrays(pose)

    print("\nInit PiDog using pose from:", POSE_FILE)
    print("LEG_PINS :", LEG_PINS,  "angles:", leg_init)
    print("HEAD_PINS:", HEAD_PINS, "angles:", head_init)
    print("TAIL_PIN :", TAIL_PIN,  "angle :", tail_init)

    dog = Pidog(
        leg_pins=LEG_PINS,
        head_pins=HEAD_PINS,
        tail_pin=TAIL_PIN,
        leg_init_angles=leg_init,
        head_init_angles=head_init,
        tail_init_angle=tail_init
    )

    if hasattr(dog, "wait_all_done"):
        dog.wait_all_done()

    # giữ nguyên logic force sau init (nếu bạn cần)
    sleep(0.2)
    force_servo_angle(FORCE_HEAD_PORT, FORCE_HEAD_ANGLE, hold=0.4)

    print("\n[DONE] Init theo pose_file xong + force head nếu cần.")


if __name__ == "__main__":
    main()
