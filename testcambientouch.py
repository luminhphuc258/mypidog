#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug cảm biến touch trên đầu PiDog:
- In liên tục giá trị cảm biến (raw + tên enum).
- Nếu phát hiện có CHẠM (khác NONE) -> phát tiếng bark ra loa PiDog.

KHÔNG tạo Pidog(), chỉ dùng DualTouch + aplay.
"""

import time
import subprocess
from pathlib import Path

# ---- ép gpiozero dùng backend RPiGPIO (ổn định hơn cho robot_hat) ----
from gpiozero import Device
from gpiozero.pins.rpigpio import RPiGPIOFactory
Device.pin_factory = RPiGPIOFactory()

from pidog.dual_touch import DualTouch, TouchStyle

# ====== CẤU HÌNH LOA & FILE BARK ======

SPEAKER_DEVICE = "plughw:3,0"   # loa PiDog (Google voiceHAT sound card)

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

    # Lấy giá trị ban đầu
    initial = touch.read()
    try:
        init_name = TouchStyle(initial).name
    except Exception:
        init_name = str(initial)

    print("\n=== DEBUG TOUCH SENSOR + BARK ===")
    print("Chạm / vuốt đầu PiDog, xem log thay đổi thế nào.")
    print("Nhấn Ctrl+C để dừng.\n")
    print(f"[INIT] trạng thái ban đầu: raw={initial}, name={init_name}\n")

    # Giá trị NONE (không chạm) trong enum (nếu có)
    try:
        NONE_VALUE = TouchStyle.NONE.value
    except Exception:
        NONE_VALUE = 0  # fallback nếu enum khác

    last_bark_time = 0.0
    DEBOUNCE_SEC = 0.5

    try:
        while True:
            val = touch.read()

            # Đổi ra tên enum nếu được
            try:
                name = TouchStyle(val).name
            except Exception:
                name = f"UNKNOWN({val})"

            # Đánh dấu nếu có chạm
            mark = "  <-- CHẠM" if val != NONE_VALUE else ""
            print(f"[TOUCH] raw={val}, name={name}{mark}")

            # Nếu KHÁC NONE thì cho sủa (có debounce)
            if val != NONE_VALUE:
                now = time.time()
                if now - last_bark_time > DEBOUNCE_SEC:
                    print("[TOUCH] -> BARK!")
                    play_bark()
                    last_bark_time = now

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng debug touch + bark.")


if __name__ == "__main__":
    main()
