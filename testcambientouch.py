#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test cảm biến touch trên đầu PiDog:
- Khi chạm vào đầu -> loa phát tiếng sủa một lần.
- Dùng DualTouch trực tiếp, KHÔNG tạo Pidog() nên không reset servo.

Nhấn Ctrl+C để dừng.
"""

import time
import subprocess
from pathlib import Path

from pidog.dual_touch import DualTouch, TouchStyle

# ==== CẤU HÌNH LOA & FILE TIẾNG SỦA ====

SPEAKER_DEVICE = "plughw:3,0"   # voiceHAT loa trên PiDog

# Thử một vài đường dẫn bark.wav phổ biến trong thư viện pidog,
# nếu không có thì fallback dùng Front_Center.wav cho chắc ăn.
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
    raise FileNotFoundError(
        "Không tìm thấy file bark.wav. "
        "Hãy sửa đường dẫn trong CANDIDATES cho đúng."
    )

print(f"[INFO] Dùng file sủa: {BARK_WAV}")


def play_bark():
    """Phát tiếng sủa ra loa PiDog."""
    cmd = ["aplay", "-D", SPEAKER_DEVICE, "-q", str(BARK_WAV)]
    subprocess.run(cmd, check=False)


def main():
    touch = DualTouch()
    last_val = 0
    last_bark_time = 0.0
    DEBOUNCE_SEC = 0.8  # tránh sủa liên tục quá nhanh

    print("\n=== TEST TOUCH + BARK ===")
    print("Chạm vào đầu PiDog (sensor) để nghe tiếng sủa.")
    print("Nhấn Ctrl+C để dừng.\n")

    try:
        while True:
            val = touch.read()  # 0 = không chạm, khác 0 = có chạm / kiểu vuốt

            if val != 0 and val != last_val:
                # chỉ sủa nếu đã cách lần trước ít nhất DEBOUNCE_SEC
                now = time.time()
                if now - last_bark_time > DEBOUNCE_SEC:
                    try:
                        name = TouchStyle(val).name
                    except Exception:
                        name = str(val)
                    print(f"[TOUCH] Phát hiện kiểu chạm: {name} ({val}) -> BARK!")
                    play_bark()
                    last_bark_time = now

            last_val = val
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng test touch + bark.")


if __name__ == "__main__":
    main()
