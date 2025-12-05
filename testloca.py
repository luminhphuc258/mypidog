#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Demo test cho PiDog (KHÔNG DÙNG Pidog()):
- Micro: ghi âm 4s từ micro, sau đó phát lại qua loa (loop liên tục).
- Cảm biến khoảng cách: nếu đến gần (< DIST_THRESHOLD_CM) thì phát tiếng "gâu gâu" qua loa.
- Cảm biến touch trên đầu: chạm vào -> LED breath màu cyan.

Nhấn Ctrl+C để dừng.
"""

import time
import threading
import subprocess
from pathlib import Path

from pidog.rgb_strip import RGBStrip
from pidog.dual_touch import DualTouch, TouchStyle

# ultrasonic: thử import từ pidog, nếu không có thì fallback sang robot_hat
try:
    from pidog.ultrasonic import Ultrasonic
except ImportError:
    try:
        from robot_hat import Ultrasonic
    except ImportError:
        Ultrasonic = None  # nếu không có thì bỏ qua test khoảng cách

# ========= CONFIG =========
# ========= CONFIG =========

AUDIO_DIR = Path.cwd() / "audio_test"
AUDIO_DIR.mkdir(exist_ok=True)

MIC_RECORD_SECONDS = 4
MIC_GAP_SECONDS = 1.0

DIST_THRESHOLD_CM = 15
SENSOR_POLL_SEC = 0.05

# 👇 ĐÃ TEST: loa phát tốt ở đây
PLAY_DEVICE = "plughw:3,0"

# 👇 Dùng device khác cho MIC (thường là default)
CAPTURE_DEVICE = "default"   # hoặc "" để không dùng -D, cho ALSA tự chọn

BARK_WAV = "/usr/share/sounds/alsa/Front_Center.wav"


# ==========================

def record_and_play(index: int) -> None:
    out_file = AUDIO_DIR / f"segment_{index:03d}.wav"
    print(f"[MIC] 🎤 Ghi âm {MIC_RECORD_SECONDS}s -> {out_file}")

    # ---- GHI ÂM (MIC) ----
    rec_cmd = [
        "arecord",
        "-f", "S16_LE",
        "-r", "16000",
        "-c", "1",
        "-d", str(MIC_RECORD_SECONDS),
        "-q",
        str(out_file),
    ]
    if CAPTURE_DEVICE:  # chỉ thêm -D nếu mình set
        rec_cmd[1:1] = ["-D", CAPTURE_DEVICE]

    try:
        subprocess.run(rec_cmd, check=False)
        print(f"[MIC] ✅ Ghi xong: {out_file}")
    except FileNotFoundError:
        print("[MIC] ❌ Không tìm thấy `arecord`. Cài: sudo apt install alsa-utils")
        return

    # ---- PHÁT LẠI (LOA) ----
    print(f"[MIC] 🔊 Phát lại: {out_file}")
    play_cmd = [
        "aplay",
        "-D", PLAY_DEVICE,
        "-q",
        str(out_file),
    ]
    try:
        subprocess.run(play_cmd, check=False)
        print("[MIC] ✅ Phát xong.")
    except FileNotFoundError:
        print("[MIC] ❌ Không tìm thấy `aplay`. Cài: sudo apt install alsa-utils")


def play_bark():
    print("[BARK] 🔊 Gâu gâu!")
    cmd = [
        "aplay",
        "-D", PLAY_DEVICE,
        "-q",
        BARK_WAV,
    ]
    try:
        subprocess.run(cmd, check=False)
    except FileNotFoundError:
        print("[BARK] ❌ Không tìm thấy `aplay` hoặc file wav.")


def read_distance_cm(ultra):
    """Đọc khoảng cách từ ultrasonic, thử cả .read() và .read_distance()."""
    if ultra is None:
        return None

    dist = None
    try:
        dist = ultra.read()
    except Exception:
        try:
            dist = ultra.read_distance()
        except Exception:
            dist = None
    return dist


def sensor_loop(ultra, dual_touch: DualTouch, strip: RGBStrip, stop_flag):
    """
    Luồng đọc cảm biến:
    - ultrasonic: nếu khoảng cách < DIST_THRESHOLD_CM -> phát tiếng 'bark'
    - touch: chạm đầu -> LED breath cyan
    """
    last_touch = None
    last_bark_time = 0

    print("[SENSOR] Bắt đầu đọc cảm biến...")

    while not stop_flag["stop"]:
        # ---- ULTRASONIC ----
        dist = read_distance_cm(ultra)
        if dist is not None and isinstance(dist, (int, float)) and 1 < dist < DIST_THRESHOLD_CM:
            now = time.time()
            if now - last_bark_time > 1.0:  # tránh bark liên tục
                print(f"[SENSOR] 🧱 Vật ở gần: {dist:.1f} cm -> bark (audio)")
                play_bark()
                last_bark_time = now

        # ---- TOUCH ----
        touch_val = None
        if dual_touch is not None:
            try:
                touch_val = dual_touch.read()
            except Exception as e:
                print("[SENSOR] Lỗi đọc touch sensor:", e)
                touch_val = None

        if touch_val is not None and touch_val != last_touch and touch_val != 0:
            try:
                style_name = TouchStyle(touch_val).name
            except Exception:
                style_name = str(touch_val)
            print(f"[SENSOR] 🤚 Touch detected: {style_name}")

            if strip is not None:
                try:
                    strip.set_mode("breath", "cyan", 1)
                    strip.show()
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
    print("[MAIN] Khởi tạo phần cứng (KHÔNG dùng Pidog)...")

    # LED strip
    try:
        strip = RGBStrip()
        strip.set_mode("breath", "yellow", 1)
        strip.show()
    except Exception as e:
        print("[MAIN] Lỗi khởi tạo RGBStrip:", e)
        strip = None

    # Ultrasonic
    if Ultrasonic is None:
        ultra = None
        print("[MAIN] Không có lớp Ultrasonic, bỏ qua cảm biến khoảng cách.")
    else:
        try:
            ultra = Ultrasonic()
        except TypeError:
            try:
                ultra = Ultrasonic()
            except Exception as e:
                print("[MAIN] Lỗi khởi tạo Ultrasonic:", e)
                ultra = None
        except Exception as e:
            print("[MAIN] Lỗi khởi tạo Ultrasonic:", e)
            ultra = None

    # Touch sensor
    try:
        dual_touch = DualTouch()
    except Exception as e:
        print("[MAIN] Lỗi khởi tạo DualTouch:", e)
        dual_touch = None

    stop_flag = {"stop": False}

    # Thread đọc cảm biến
    t_sensor = threading.Thread(
        target=sensor_loop,
        args=(ultra, dual_touch, strip, stop_flag),
        daemon=True,
    )
    # Thread test micro (ghi + phát)
    t_mic = threading.Thread(
        target=mic_test_loop,
        args=(stop_flag,),
        daemon=True,
    )

    t_sensor.start()
    t_mic.start()

    print(
        "\n[MAIN] Demo đang chạy (KHÔNG reset servo vì không tạo Pidog()):\n"
        "  - Micro: ghi 4s rồi phát lại qua loa (loop) -> ./audio_test\n"
        f"  - Cảm biến khoảng cách (nếu Ultrasonic OK): < {DIST_THRESHOLD_CM} cm -> phát BARK_WAV\n"
        "  - Touch đầu (nếu DualTouch OK): chạm -> LED breath cyan\n"
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

        if strip is not None:
            try:
                strip.set_mode("breath", [0, 0, 0], 1, brightness=0)
                strip.show()
                strip.close()
            except Exception:
                pass

        print("[MAIN] Thoát demo (servo KHÔNG bị reset vì không dùng Pidog()).")


if __name__ == "__main__":
    main()
