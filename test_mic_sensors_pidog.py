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

# KHÔNG import Pidog
# from pidog.pidog import Pidog

# Dùng module con, không đụng tới servo
from pidog.rgb_strip import RGBStrip
from pidog.dual_touch import DualTouch, TouchStyle

# ultrasonic: thử import từ pidog, nếu không có thì fallback sang robot_hat
try:
    from pidog.ultrasonic import Ultrasonic
except ImportError:
    try:
        from robot_hat import Ultrasonic
    except ImportError:
        Ultrasonic = None  # không có ultrasonic, sẽ bỏ qua phần test khoảng cách

# ========= CONFIG =========

AUDIO_DIR = Path.cwd() / "audio_test"
AUDIO_DIR.mkdir(exist_ok=True)

MIC_RECORD_SECONDS = 4        # ghi 4 giây
MIC_GAP_SECONDS = 1.0         # nghỉ 1 giây rồi ghi tiếp

DIST_THRESHOLD_CM = 15        # khoảng cách để "sủa"
SENSOR_POLL_SEC = 0.05        # chu kỳ đọc cảm biến

# Dùng device ALSA "default" (PiDog thường cấu hình sẵn về đúng sound card)
ALSA_DEVICE = "default"

# File âm thanh tiếng sủa (bạn có thể đổi sang file riêng)
BARK_WAV = "/usr/share/sounds/alsa/Front_Center.wav"

# ==========================


def record_and_play(index: int) -> None:
    """
    Ghi 1 đoạn audio từ micro của PiDog bằng `arecord`,
    sau đó phát lại bằng `aplay`, dùng ALSA device "default".
    Ghi ở 16kHz mono cho hợp với config của PiDog.
    """
    out_file = AUDIO_DIR / f"segment_{index:03d}.wav"
    print(f"[MIC] 🎤 Ghi âm {MIC_RECORD_SECONDS}s -> {out_file}")

    rec_cmd = [
        "arecord",
        "-D", ALSA_DEVICE,      # dùng ALSA default
        "-f", "S16_LE",         # 16-bit
        "-r", "16000",          # 16kHz
        "-c", "1",              # mono
        "-d", str(MIC_RECORD_SECONDS),
        "-q",
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
        "-D", ALSA_DEVICE,
        "-q",
        str(out_file),
    ]
    try:
        subprocess.run(play_cmd, check=False)
        print("[MIC] ✅ Phát xong.")
    except FileNotFoundError:
        print("[MIC] ❌ Không tìm thấy `aplay`. Cài: sudo apt install alsa-utils")


def play_bark():
    """
    Phát tiếng "gâu gâu" qua loa.
    Bạn có thể đổi BARK_WAV thành file tiếng chó riêng.
    """
    print("[BARK] 🔊 Gâu gâu!")
    cmd = [
        "aplay",
        "-D", ALSA_DEVICE,
        "-q",
        BARK_WAV,
    ]
    try:
        subprocess.run(cmd, check=False)
    except FileNotFoundError:
        print("[BARK] ❌ Không tìm thấy `aplay` hoặc file wav.")


def read_distance_cm(ultra):
    """
    Đọc khoảng cách từ ultrasonic.
    Thử cả 2 kiểu API: .read() và .read_distance().
    """
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
    - ultrasonic: nếu khoảng cách < DIST_THRESHOLD_CM -> phát tiếng "bark"
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

            # chạm -> bật LED breath cyan
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
            # Nếu bản của bạn cần tham số pin thì chỉnh lại ở đây
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
