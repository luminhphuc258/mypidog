#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import subprocess
from pathlib import Path

from pidog.dual_touch import DualTouch, TouchStyle

# ====== LOA + FILE BARK ======
SPEAKER_DEVICE = "plughw:3,0"

CANDIDATES = [
    Path("/usr/local/lib/python3.11/dist-packages/pidog/res/bark.wav"),
    Path("/usr/local/lib/python3.9/dist-packages/pidog/res/bark.wav"),
    Path("/usr/local/lib/python3.11/dist-packages/pidog/res/dog_bark.wav"),
    Path("/usr/share/sounds/alsa/Front_Center.wav"),
]

BARK_WAV = None
for p in CANDIDATES:
    if p.exists():
        BARK_WAV = p
        break

if BARK_WAV is None:
    raise FileNotFoundError("Không tìm thấy file bark.wav, hãy sửa CANDIDATES.")

print(f"[INFO] Dùng file sủa: {BARK_WAV}")


def play_bark():
    cmd = ["aplay", "-D", SPEAKER_DEVICE, str(BARK_WAV)]
    print("[BARK] run:", " ".join(cmd))
    try:
        r = subprocess.run(cmd)
        print("[BARK] exit code:", r.returncode)
    except Exception as e:
        print("[BARK] Lỗi aplay:", e)


def main():
    touch = DualTouch()

    init_val = touch.read()
    try:
        init_name = TouchStyle(init_val).name
    except Exception:
        init_name = str(init_val)

    print("\n=== DEBUG TOUCH SENSOR + BARK (NO Pidog) ===")
    print("Chạm / vuốt đầu PiDog, xem raw & name có đổi không.")
    print("Nhấn Ctrl+C để dừng.\n")
    print(f"[INIT] raw={init_val}, name={init_name}\n")

    # NONE value trong enum (nếu có)
    try:
        NONE_VALUE = TouchStyle.NONE.value
    except Exception:
        NONE_VALUE = init_val  # fallback: coi trạng thái ban đầu là NONE

    last_val = init_val
    last_bark_time = 0
    DEBOUNCE = 0.5

    try:
        while True:
            val = touch.read()
            try:
                name = TouchStyle(val).name
            except Exception:
                name = f"UNKNOWN({val})"

            changed = (val != last_val)
            mark = ""
            if changed:
                mark = "  <-- CHANGED"
            print(f"[TOUCH] raw={val}, name={name}{mark}")

            # chỉ bark khi chuyển từ NONE -> KHÁC NONE
            if last_val == NONE_VALUE and val != NONE_VALUE:
                now = time.time()
                if now - last_bark_time > DEBOUNCE:
                    print("[TOUCH] TRANSITION NONE -> TOUCHED -> BARK!")
                    play_bark()
                    last_bark_time = now

            last_val = val
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng debug touch.")


if __name__ == "__main__":
    main()
