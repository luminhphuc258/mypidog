#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- Unlock Robot-HAT/PiDog speaker via SPK_EN GPIO (như i2samp.sh)
- Init Pidog theo pose bạn set
- Sau init: ép xoay head servo (bypass giới hạn pidog) về -90 bằng robot_hat.Servo
"""

import os
import shutil
import subprocess
from time import sleep
from pidog import Pidog
from robot_hat import Servo

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

LEG_INIT_ANGLES  = [-3, 89, 9, -80, 3, 90, 10, -90]
HEAD_INIT_ANGLES = [20, -45, -90]   # pose init bạn muốn
TAIL_INIT_ANGLE  = [30]             # MUST be list

# ===================== FORCE HEAD SERVO AFTER INIT =====================
# Mình hiểu “head index 3” bạn nói = HEAD_YAW (P10). Nếu muốn đổi, sửa ở đây:
FORCE_HEAD_PORT  = "P10"
FORCE_HEAD_ANGLE = -90

def force_servo_angle(port: str, angle: float, hold=0.3):
    # clamp an toàn theo robot_hat (-90..90)
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

    # Quan trọng: ép servo head sau init (bỏ qua giới hạn pidog)
    sleep(0.2)
    force_servo_angle(FORCE_HEAD_PORT, FORCE_HEAD_ANGLE, hold=0.4)

    print("\n[DONE] Init xong + đã force head servo.")


if __name__ == "__main__":
    main()
