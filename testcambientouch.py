#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test cảm biến touch trên đầu PiDog:
- mỗi lần giá trị touch thay đổi -> in ra và phát tiếng sủa.

KHÔNG dùng Pidog(), chỉ DualTouch + aplay.
"""

import time
import subprocess
from pathlib import Path

# ---- ép gpiozero dùng RPiGPIO (tránh backend lgpio chiếm GPIO) ----
from gpiozero import Device
from gpiozero.pins.rpigpio import RPiGPIOFactory
Device.pin_factory = RPiGPIOFactory()

from pidog.dual_touch import DualTouch, TouchStyle

# ====== CẤU HÌNH LOA & FILE BARK ======

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
    """Phát tiếng sủa ra loa PiDog & in log chi tiết."""
    cmd = ["aplay", "-D", SPEAKER_DEVICE, str(BARK_WAV)]
    print("[BARK] chạy lệnh:", " ".join(cmd))
    try:
        res = subprocess.run(cmd)
        print("[BARK] aplay exit code:", res.returncode)
    except Exception as e:
        print("[BARK] Lỗi khi gọi aplay:", e)


def main():
    touch = DualTouch()

    # Đọc giá trị ban đầu, chỉ log, KHÔNG bark
    initial = touch.read()
    try:
        init_name = TouchStyle(initial).name
    except Exception:
        init_name = str(initial)
    print(f"\n=== TEST TOUCH + BARK ===")
    print("Chạm vào đầu PiDog để nghe tiếng sủa.")
    print("Nhấn Ctrl+C để dừng.")
    print(f"\n[INIT] trạng thái ban đầu: {init_name} ({initial})\n")

    last_val = initial
    last_bark_time = 0.0
    DEBOUNCE_SEC = 0.5

    try:
        while True:
            val = touch.read()

            if val != last_val:
                try:
                    name = TouchStyle(val).name
                except Exception:
                    name = str(val)
                print(f"[TOUCH] giá trị thay đổi: {name} ({val})")

                # CHỈ bark khi có chạm (val != 0) và không phải NONE
                if val != 0 and name != "NONE":
                    now = time.time()
                    if now - last_bark_time > DEBOUNCE_SEC:
                        print("[TOUCH] -> BARK!")
                        play_bark()
                        last_bark_time = now

                last_val = val

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng test touch + bark.")



if __name__ == "__main__":
    main()
