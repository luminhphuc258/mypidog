#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Demo test cho PiDog:
- Micro: ghi âm 4s từ micro, sau đó phát lại qua loa (loop liên tục).
- Cảm biến khoảng cách: nếu đến gần (< DIST_THRESHOLD_CM) thì robot bark.
- Cảm biến touch trên đầu: chạm vào -> LED breath màu cyan.

Nhấn Ctrl+C để dừng.
"""

import time
import threading
import subprocess
from pathlib import Path

from pidog.pidog import Pidog
from pidog.dual_touch import TouchStyle

# ========= CONFIG =========

AUDIO_DIR = Path.cwd() / "audio_test"
AUDIO_DIR.mkdir(exist_ok=True)

MIC_RECORD_SECONDS = 4        # ghi 4 giây
MIC_GAP_SECONDS = 1.0         # nghỉ 1 giây rồi ghi tiếp

DIST_THRESHOLD_CM = 15        # khoảng cách để bark
SENSOR_POLL_SEC = 0.05        # chu kỳ đọc cảm biến

# ==========================


def record_and_play(index: int) -> None:
    """
    Ghi 1 đoạn audio từ micro của PiDog bằng `arecord`,
    sau đó phát lại bằng `aplay`.
    """
    out_file = AUDIO_DIR / f"segment_{index:03d}.wav"
    print(f"[MIC] 🎤 Ghi âm {MIC_RECORD_SECONDS}s -> {out_file}")

    # Ghi âm
    rec_cmd = [
        "arecord",
        "-f", "cd",                 # 16-bit, 44.1kHz, stereo
        "-d", str(MIC_RECORD_SECONDS),
        "-q",                       # quiet
        str(out_file),
    ]

    try:
        subprocess.run(rec_cmd, check=False)
        print(f"[MIC] ✅ Ghi xong: {out_file}")
    except FileNotFoundError:
        print("[MIC] ❌ Không tìm thấy `arecord`. Cài: sudo apt install alsa-utils")
        return

    # Phát lại
    print(f"[MIC] 🔊 Phát lại: {out_file}")
    play_cmd = [
        "aplay",
        "-q",               # quiet
        str(out_file),
    ]
    try:
        subprocess.run(play_cmd, check=False)
        print("[MIC] ✅ Phát xong.")
    except FileNotFoundError:
        print("[MIC] ❌ Không tìm thấy `aplay`. Cài: sudo apt install alsa-utils")


def sensor_loop(dog: Pidog, stop_flag):
    """
    Luồng đọc cảm biến:
    - ultrasonic: nếu khoảng cách < DIST_THRESHOLD_CM -> bark
    - touch: chạm đầu -> LED breath cyan
    """
    last_touch = None
    last_bark_time = 0

    print("[SENSOR] Bắt đầu đọc cảm biến...")

    while not stop_flag["stop"]:
        # ---- ULTRASONIC ----
        try:
            dist = dog.read_distance()
        except Exception as e:
            print("[SENSOR] Lỗi đọc khoảng cách:", e)
            dist = None

        if dist is not None and 1 < dist < DIST_THRESHOLD_CM:
            now = time.time()
            if now - last_bark_time > 1.0:  # tránh bark liên tục
                print(f"[SENSOR] 🧱 Vật ở gần: {dist:.1f} cm -> bark")
                try:
                    dog.do_action("bark", speed=80)
                except Exception as e:
                    print("[SENSOR] Lỗi dog.do_action('bark'):", e)
                last_bark_time = now

        # ---- TOUCH ----
        try:
            touch_val = dog.dual_touch.read()
        except Exception as e:
            print("[SENSOR] Lỗi đọc touch sensor:", e)
            touch_val = None

        if touch_val is not None and touch_val != last_touch and touch_val != 0:
            try:
                style_name = TouchStyle(touch_val).name
            except Exception:
                style_name = str(touch_val)
            print(f"[SENSOR] 🤚 Touch detected: {style_name}")

            # chạm -> bật LED breath cyan
            try:
                dog.rgb_strip.set_mode("breath", "cyan", 1)
            except Exception as e:
                print("[SENSOR] Lỗi set_mode rgb_strip:", e)

        last_touch = touch_val

        time.sleep(SENSOR_POLL_SEC)

    print("[SENSOR] Dừng sensor loop.")


def mic_test_loop(stop_flag):
    """
    Luồng test micro:
    - Ghi 4s
    - Phát lại
    - Nghỉ 1s
    - Lặp lại cho đến khi stop_flag["stop"] = True
    """
    i = 0
    print("[MIC] Bắt đầu loop ghi + phát lại (4s mỗi lần).")
    while not stop_flag["stop"]:
        record_and_play(i)
        i += 1

        # nghỉ giữa 2 lần test
        total_wait = MIC_GAP_SECONDS
        step = 0.1
        waited = 0.0
        while waited < total_wait and not stop_flag["stop"]:
            time.sleep(step)
            waited += step

    print("[MIC] Dừng mic loop.")


def main():
    print("[MAIN] Khởi tạo PiDog...")
    dog = Pidog()
    time.sleep(1)

    # Đưa robot về tư thế SIT + bật LED vàng nhẹ để biết demo đang chạy
    try:
        dog.do_action("sit", speed=60)
    except Exception:
        pass

    try:
        dog.rgb_strip.set_mode("breath", "yellow", 1)
    except Exception:
        pass

    stop_flag = {"stop": False}

    # 1 thread đọc cảm biến
    t_sensor = threading.Thread(target=sensor_loop, args=(dog, stop_flag), daemon=True)
    # 1 thread test micro (ghi + phát)
    t_mic = threading.Thread(target=mic_test_loop, args=(stop_flag,), daemon=True)

    t_sensor.start()
    t_mic.start()

    print(
        "\n[MAIN] Demo đang chạy:\n"
        "  - Micro: ghi 4s rồi phát lại qua loa (lặp liên tục) vào thư mục ./audio_test\n"
        f"  - Cảm biến khoảng cách: nếu < {DIST_THRESHOLD_CM} cm -> bark\n"
        "  - Touch đầu: chạm -> LED breath cyan\n"
        "Nhấn Ctrl+C để dừng.\n"
    )

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[MAIN] Nhận Ctrl+C, đang dừng demo...")
    finally:
        stop_flag["stop"] = True
        time.sleep(1.0)
        # tắt LED + đóng dog
        try:
            dog.rgb_strip.set_mode("breath", [0, 0, 0], 1, brightness=0)
            dog.rgb_strip.show()
            dog.rgb_strip.close()
        except Exception:
            pass
        try:
            dog.close()
        except Exception:
            pass
        print("[MAIN] Thoát demo.")


if __name__ == "__main__":
    main()
