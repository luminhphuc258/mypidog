#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
unlock_khoitaoservomotor.py
- Unlock Robot-HAT/PiDog speaker via SPK_EN GPIO (như i2samp.sh)
- Init Pidog với pose chân đúng (P0..P7) & head (P8..P10) theo mapping joint-name
- Fix tail_init_angle: phải là LIST (dài = số pin tail)
- Fix head bị nghiêng: set NECK_TILT(P9)=0 để đầu thẳng

Chạy:
  sudo python3 unlock_khoitaoservomotor.py
"""

import os
import shutil
import subprocess
from time import sleep
from pidog import Pidog


# ===================== AUDIO UNLOCK (SPK_EN) =====================

SPEAKER_DEVICE = "plughw:3,0"  # bạn đang dùng cái này trong project active listening

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
    """
    Theo i2samp.sh:
      - googlevoicehat-soundcard (with mic)  -> SPK_EN = GPIO12
      - hifiberry-dac (without mic)          -> SPK_EN = GPIO20
    """
    txt = _read_boot_config()
    lines = [ln.split("#", 1)[0].strip() for ln in txt.splitlines()]
    overlays = [ln for ln in lines if ln.startswith("dtoverlay=")]

    if any("googlevoicehat-soundcard" in ln for ln in overlays):
        return 12
    if any("hifiberry-dac" in ln for ln in overlays):
        return 20

    # fallback: nhìn aplay -l có card googlevoicehat không
    try:
        out = subprocess.run(["aplay", "-l"], capture_output=True, text=True).stdout.lower()
        if "googlevoi" in out or "googlevoicehat" in out:
            return 12
    except:
        pass

    return 20  # fallback cuối

def set_gpio_high(pin: int) -> bool:
    if shutil.which("pinctrl"):
        subprocess.run(["pinctrl", "set", str(pin), "op", "dh"], check=False)
        return True
    if shutil.which("raspi-gpio"):
        subprocess.run(["raspi-gpio", "set", str(pin), "op", "dh"], check=False)
        return True
    return False

def prime_speaker(device: str):
    # phát 0.1s silence để "prime" đường audio (giống play -n trim 0 0.5)
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

    # prime để chắc chắn amp đã bật
    prime_speaker(device)

    # best-effort set volume, không bắt buộc
    subprocess.run(["amixer", "sset", "robot-hat speaker", "100%"], check=False)
    subprocess.run(["amixer", "sset", "robot-hat speaker Playback Volume", "100%"], check=False)

    print(f"[OK] Speaker unlocked (SPK_EN GPIO{pin})")
    return True


# ===================== PIDOG INIT POSE =====================

# Mapping bạn xác nhận:
# FL_HIP=P0, FL_LOWER=P1, FR_HIP=P2, FR_LOWER=P3,
# RL_HIP=P4, RL_LOWER=P5, RR_HIP=P6, RR_LOWER=P7,
# NECK_PITCH=P8, NECK_TILT=P9, HEAD_YAW=P10, TAIL=P11

LEG_PINS  = [0, 1, 2, 3, 4, 5, 6, 7]
HEAD_PINS = [8, 9, 10]
TAIL_PIN  = [11]

# Pose chân theo ảnh bạn gửi:
LEG_INIT_ANGLES = [-3, 89, 9, -80, 3, 90, 10, -90]

# Head theo mapping mới: [NECK_PITCH(P8), NECK_TILT(P9), HEAD_YAW(P10)]
# Bạn bị "ngã trái 90" là do P9=+90 -> tilt max.
# Cho đầu thẳng: set tilt = 0
# index 0 la phan neck pitch, index 1 la co qua phai trai, mac dinh la -90 nhin thang, index 2 là co qua truoc sai
HEAD_INIT_ANGLES = [90, -90, -90]

# Tail: PHẢI là list (số phần tử = số pin tail)
TAIL_INIT_ANGLE = [0]


def main():
    print("=== Unlock speaker + Init PiDog with mapped pose ===")
    print(f"Speaker device: {SPEAKER_DEVICE}")
    unlock_robothat_speaker(SPEAKER_DEVICE)

    print("\nInit PiDog with mapped pose...")
    print("LEG_PINS :", LEG_PINS, "angles:", LEG_INIT_ANGLES)
    print("HEAD_PINS:", HEAD_PINS, "angles:", HEAD_INIT_ANGLES)
    print("TAIL_PIN :", TAIL_PIN, "angle :", TAIL_INIT_ANGLE)

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

    sleep(1.0)
    print("\n[DONE] Nếu chân đúng & đầu thẳng, pose init đã OK.")


if __name__ == "__main__":
    main()
