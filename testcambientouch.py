#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test cảm biến touch trên đầu PiDog:
- Chạm vào đầu -> loa PiDog phát tiếng sủa một lần.

KHÔNG dùng Pidog(), chỉ dùng DualTouch + aplay.
"""

import time
import subprocess
from pathlib import Path

# ---- ép gpiozero dùng RPiGPIO thay vì lgpio (tránh lỗi GPIO busy) ----
from gpiozero import Device
from gpiozero.pins.rpigpio import RPiGPIOFactory
Device.pin_factory = RPiGPIOFactory()

from pidog.dual_touch import DualTouch, TouchStyle

# ========= CẤU HÌNH LOA & FILE BARK =========

SPEAKER_DEVICE = "plughw:3,0"   # loa PiDog (voiceHAT)

CANDIDATES = [
    Path("/usr/local/lib/python3.11/dist-packages/pidog/res/bark.wav"),
    Path("/usr/local/lib/python3.9/dist-packages/pidog/res/bark.wav"),
    Path("/usr/local/lib/python3.11/dist-packages/pidog/res/dog_bark.wav"),
    Path("/usr/share/sounds/alsa/Front_Center.wav"),  # fallback
]

BARK_WAV = None
for p in CANDIDATES:
    if p.exists():
        BARK_WAV = p
        break

if BARK_WAV is None:
    raise FileNotFoundError("Không tìm thấy file bark.wav, hãy sửa lại CANDIDATES.")

print(f"[INFO] Dùng file sủa: {BARK_WAV}")


def play_bark():
    """Phát tiếng sủa ra loa PiDog."""
    cmd = ["aplay", "-D", SPEAKER_DEVICE, "-q", str(BARK_WAV)]
    subprocess.run(cmd, check=False)


def main():
    # tạo đối tượng đọc cảm biến touch
    touch = DualTouch()

    last_val = 0
    last_bark_time = 0.0
    DEBOUNCE_SEC = 0.8  # tránh sủa quá dày

    print("\n=== TEST TOUCH + BARK ===")
    print("Chạm vào đầu PiDog để nghe tiếng sủa.")
    print("Nhấn Ctrl+C để dừng.\n")

    try:
        while True:
            val = touch.read()   # 0 = không chạm, khác 0 = có chạm

            if val != 0 and val != last_val:
                now = time.time()
                if now - last_bark_time > DEBOUNCE_SEC:
                    try:
                        name = TouchStyle(val).name
                    except Exception:
                        name = str(val)
                    print(f"[TOUCH] kiểu chạm: {name} ({val}) -> BARK!")
                    play_bark()
                    last_bark_time = now

            last_val = val
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng test touch + bark.")


if __name__ == "__main__":
    main()
