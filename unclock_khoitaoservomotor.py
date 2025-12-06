#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- Unlock Robot-HAT/PiDog speaker via SPK_EN GPIO
- Load pose từ pidog_pose_config.txt (format như servo editor: P0..P11)
- Remap head: file(P8 yaw, P9 roll, P10 pitch) -> pidog(P8 pitch, P9 tilt, P10 yaw)
- Init Pidog bằng init_angles đã remap
- Giữ nguyên: force head servo sau init (bypass pidog limit)
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


# ===================== CONFIG =====================

SPEAKER_DEVICE = "plughw:3,0"
POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"

# Pidog pins (theo mapping bạn đã confirm)
LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
HEAD_PINS = [8, 9, 10]   # P8=NECK_PITCH, P9=NECK_TILT, P10=HEAD_YAW
TAIL_PIN  = [11]

# Force head sau init (giữ nguyên như bạn muốn)
FORCE_HEAD_PORT  = "P10"
FORCE_HEAD_ANGLE = -90


# ===================== POSE LOADER (FILE FORMAT = SERVO EDITOR) =====================

def load_pose_angles_robot_hat_style(path: Path) -> dict[int, float]:
    """
    Đọc pose file kiểu servo editor: P0..P11.
    Hỗ trợ:
      - JSON: {"P0": -3, "P1": 89, ...}
      - Text: P0 : -3   /  P1=89  /  P10  -90
    Trả về dict {0..11: angle}
    """
    txt = path.read_text(encoding="utf-8", errors="ignore").strip()

    # thử JSON
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

        # match "P0 : -3" / "P1=89" / "P10  -90"
        m = re.search(r"[Pp]\s*(\d+)\s*[:=]?\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            ch = int(m.group(1))
            ang = float(m.group(2))
            out[ch] = ang
            continue

    return out


def build_pidog_init_angles_from_posefile(pose: dict[int, float]):
    """
    pose: theo robot_hat file (P8=yaw, P9=roll, P10=pitch)
    pidog wants:
      - legs: P0..P7 giữ nguyên
      - head: P8=pitch, P9=tilt, P10=yaw  (REMAPPED!)
      - tail: P11 giữ nguyên (nhưng phải là list)
    """
    need = list(range(12))
    missing = [i for i in need if i not in pose]
    if missing:
        raise ValueError(f"Pose file thiếu channel: {missing} (cần đủ P0..P11)")

    leg_init = [pose[p] for p in LEG_PINS]

    # REMAP HEAD:
    # file: P8 yaw, P9 roll, P10 pitch
    # pidog: P8 pitch, P9 tilt, P10 yaw
    head_pitch = pose[10]   # file P10 -> pidog P8
    head_tilt  = pose[9]    # file P9  -> pidog P9
    head_yaw   = pose[8]    # file P8  -> pidog P10

    head_init = [head_pitch, head_tilt, head_yaw]

    tail_init = [pose[11]]  # MUST be list

    return leg_init, head_init, tail_init


# ===================== AUDIO UNLOCK (SPK_EN) =====================

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


# ===================== FORCE SERVO (BYPASS PIDOG) =====================

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
    print("=== Unlock speaker + Init Pidog from pose file (with remap) ===")
    unlock_robothat_speaker(SPEAKER_DEVICE)

    if not POSE_FILE.exists():
        raise FileNotFoundError(f"Không thấy file pose: {POSE_FILE}")

    pose = load_pose_angles_robot_hat_style(POSE_FILE)
    leg_init, head_init, tail_init = build_pidog_init_angles_from_posefile(pose)

    print("\nPose file:", POSE_FILE)
    print("LEG_INIT :", leg_init)
    print("HEAD_INIT (pidog order pitch,tilt,yaw):", head_init, "  (remap from file P10,P9,P8)")
    print("TAIL_INIT:", tail_init)

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

    # giữ nguyên: force head yaw sau init nếu cần
    sleep(0.2)
    force_servo_angle(FORCE_HEAD_PORT, FORCE_HEAD_ANGLE, hold=0.4)

    print("\n[DONE] Init theo file + remap xong.")

if __name__ == "__main__":
    main()
