#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pidog_camera_avoid.py

Flow:
1) Apply pose chuẩn từ pidog_pose_config.txt (robot_hat only) -> mượt, chậm
2) Boot Pidog bằng MatthewPidogBootClass (bạn đã test OK)
3) Loop:
   - Forward bình thường
   - Camera detect vật cản (dựa trên color detect của Vilib)
        Nếu thấy vật cản:
            Stand yên 2s -> lùi -> rẽ (mặc định phải) -> đi tiếp
            Nếu quá gần -> bark 3 tiếng
   - IMU detect va chạm:
        Lùi 5s -> rẽ phải -> đi tiếp
4) Ctrl+C để thoát -> trả pose config -> close

Ghi log:
- console + file avoid_log.csv

Lưu ý:
- Bạn cần chạy sudo:
    sudo python3 pidog_camera_avoid.py
"""

import json
import csv
import time
import math
from pathlib import Path
from time import sleep

from robot_hat import Servo
from pidog.preset_actions import bark
from vilib import Vilib

from matthewpidogclassinit import MatthewPidogBootClass


# ===================== USER CONFIG =====================

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
LOG_FILE  = Path(__file__).resolve().parent / "avoid_log.csv"

SERVO_PORTS = [f"P{i}" for i in range(12)]
ANGLE_MIN, ANGLE_MAX = -90, 90

# apply pose mượt
POSE_STEP_DELAY = 0.012   # càng lớn càng "êm" nhưng chậm
POSE_SETTLE_SEC = 1.0

# tốc độ hành động pidog
SPEED_STAND   = 25
SPEED_FORWARD = 220
SPEED_TURN    = 200
SPEED_BACK    = 220

# thời lượng né vật cản
HOLD_STAND_SEC      = 2.0
BACKUP_SEC_OBSTACLE = 1.8

# va chạm: lùi 5s
BACKUP_SEC_BUMP = 5.0

# ===== Camera obstacle detection tuning =====
# Vilib color detect: color_n, color_x, color_w, color_h
# "diện tích" = w*h (càng lớn càng gần)
AREA_SEEN  = 9000     # thấy đáng kể
AREA_CLOSE = 22000    # quá gần -> bark

# nếu muốn quyết định trái/phải theo vị trí vật cản trong khung hình
# (x < center => vật cản bên trái => rẽ phải, và ngược lại)
USE_X_TO_DECIDE = True
FRAME_W = 640
CENTER_X = FRAME_W // 2
X_DEADBAND = 60

# nếu không dùng X để quyết định, dùng hướng cố định
TURN_PREFERENCE = "right"  # "right" hoặc "left"

# turn strength
TURN_STEPS_OBSTACLE = 3     # rẽ nhẹ
TURN_STEPS_BUMP     = 4     # va chạm thì rẽ mạnh hơn

# ===== Head lock (giữ đầu ổn định, optional) =====
LOCK_HEAD = True
LOCK_P8 = 32     # bạn muốn giữ P8 = 32
LOCK_P9 = -90    # cổ
# P10 lắc nhẹ để “nhìn”
WIGGLE_P10 = True
P10_A, P10_B = -70, -89
WIGGLE_INTERVAL = 0.10
WIGGLE_HOLD_MIN = 0.7
WIGGLE_HOLD_MAX = 1.6


# ===================== UTILS =====================

def clamp(v, lo=ANGLE_MIN, hi=ANGLE_MAX):
    try:
        v = int(v)
    except Exception:
        v = 0
    return max(lo, min(hi, v))

def servo_set_angle(servo_obj, angle: int):
    angle = clamp(angle)
    for method_name in ("angle", "write", "set_angle", "setAngle"):
        m = getattr(servo_obj, method_name, None)
        if callable(m):
            m(angle)
            return
    if callable(servo_obj):
        servo_obj(angle)
        return
    raise RuntimeError("Servo object has no known angle/write method")

def load_pose_config(path: Path) -> dict:
    cfg = {p: 0 for p in SERVO_PORTS}
    if not path.exists():
        print(f"[WARN] Pose file not found: {path} -> use all zeros.")
        return cfg
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            for k, v in data.items():
                if k in cfg:
                    cfg[k] = clamp(v)
    except Exception as e:
        print(f"[WARN] Pose file parse error: {e} -> use all zeros.")
    return cfg

def apply_pose_config(cfg: dict, step_delay=POSE_STEP_DELAY, settle_sec=POSE_SETTLE_SEC):
    """
    Apply pose to all servos via robot_hat only (NO pidog).
    """
    print("[STEP1/STEP4] Apply pose from config file (robot_hat.Servo)...")

    servos = {}
    for p in SERVO_PORTS:
        try:
            servos[p] = Servo(p)
        except Exception as e:
            print(f"[WARN] Cannot init Servo({p}): {e}")

    for p in SERVO_PORTS:
        if p not in servos:
            continue
        try:
            servo_set_angle(servos[p], cfg.get(p, 0))
            time.sleep(step_delay)
        except Exception as e:
            print(f"[WARN] Apply {p} failed: {e}")

    if settle_sec and settle_sec > 0:
        print(f"[STABLE] settle {settle_sec:.1f}s ...")
        time.sleep(settle_sec)

def open_log_file(path: Path):
    new_file = not path.exists()
    f = open(path, "a", newline="", encoding="utf-8")
    w = csv.writer(f)
    if new_file:
        w.writerow(["ts", "event", "detail", "color_n", "color_x", "color_w", "color_h", "area", "turn_dir"])
        f.flush()
    return f, w

def log_event(writer, event, detail="", color=None, turn_dir=""):
    ts = time.time()
    if color is None:
        row = [ts, event, detail, "", "", "", "", "", turn_dir]
    else:
        n,x,w,h,area = color
        row = [ts, event, detail, n, x, w, h, area, turn_dir]
    writer.writerow(row)

def safe_bark_3(dog):
    try:
        for _ in range(3):
            bark(dog, [0, 0, 0], volume=80)
            sleep(0.15)
    except:
        # fallback nếu bark action lỗi
        try:
            dog.do_action("bark", step_count=1, speed=80)
            dog.wait_all_done()
        except:
            pass

def do_backward_for_seconds(dog, seconds=1.8, speed=220):
    t0 = time.time()
    while time.time() - t0 < seconds:
        dog.do_action("backward", step_count=1, speed=speed)
        dog.wait_all_done()

def decide_turn_dir_from_x(x):
    """
    Nếu vật cản nằm bên trái khung hình (x < center) -> rẽ phải để tránh.
    Nếu vật cản bên phải -> rẽ trái.
    """
    if x is None:
        return TURN_PREFERENCE
    dx = x - CENTER_X
    if abs(dx) < X_DEADBAND:
        return TURN_PREFERENCE
    return "left" if dx > 0 else "right"

def do_turn(dog, direction, steps, speed):
    if direction == "left":
        dog.do_action("turn_left", step_count=steps, speed=speed)
    else:
        dog.do_action("turn_right", step_count=steps, speed=speed)
    dog.wait_all_done()


# ===================== CAMERA DETECT (VILIB) =====================

def camera_start_web():
    # Bật camera + web stream
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)

    # Bật color detect (dùng để làm "obstacle marker")
    # Bạn có thể chọn màu trong Vilib (tuỳ version).
    # Một số bản cần thêm Vilib.color_detect_switch(True)
    try:
        Vilib.color_detect_switch(True)
    except:
        # nếu không có, vẫn đọc dict được nhưng phải bật mode khác
        pass

    sleep(0.3)

def camera_read_color():
    """
    Return: (n, x, w, h, area)
    - n: số object detect
    - x: center x (0..640)
    - w,h: bbox width/height
    - area: w*h
    """
    d = Vilib.detect_obj_parameter
    n = int(d.get("color_n", 0) or 0)
    x = d.get("color_x", None)
    w = d.get("color_w", 0) or 0
    h = d.get("color_h", 0) or 0
    try:
        x = int(x) if x is not None else None
    except:
        x = None
    area = int(w) * int(h)
    return n, x, int(w), int(h), area


# ===================== IMU BUMP DETECT =====================

def imu_bump_detect(dog, accel_threshold=1.65, jerk_threshold=1.2):
    """
    Cách đơn giản:
    - đọc accel (ax,ay,az) nếu có
    - tính độ lớn g = sqrt(ax^2+ay^2+az^2)
    - nếu g tăng đột ngột (jerk) hoặc vượt ngưỡng -> coi như va chạm
    """
    if not hasattr(dog, "imu"):
        return False

    try:
        imu = dog.imu
        # tuỳ version: imu.get_accel() / imu.accel / imu.read_accel ...
        ax = ay = az = None

        if hasattr(imu, "get_accel"):
            ax, ay, az = imu.get_accel()
        elif hasattr(imu, "accel"):
            ax, ay, az = imu.accel
        elif hasattr(imu, "read_accel"):
            ax, ay, az = imu.read_accel()
        else:
            return False

        g = math.sqrt(ax*ax + ay*ay + az*az)
        # lưu g trước đó
        if not hasattr(imu_bump_detect, "_prev_g"):
            imu_bump_detect._prev_g = g
            return False

        prev = imu_bump_detect._prev_g
        imu_bump_detect._prev_g = g

        jerk = abs(g - prev)
        if g >= accel_threshold or jerk >= jerk_threshold:
            return True
        return False
    except:
        return False


# ===================== HEAD LOCK (OPTIONAL) =====================

import threading
import random

def start_head_lock():
    if not LOCK_HEAD:
        return None, None

    stop_evt = threading.Event()
    try:
        s8 = Servo("P8")
        s9 = Servo("P9")
        s10 = Servo("P10")
    except Exception as e:
        print(f"[WARN] Cannot init head servos P8/P9/P10: {e}")
        return None, None

    def worker():
        # init
        try:
            s8.angle(clamp(LOCK_P8))
            s9.angle(clamp(LOCK_P9))
            if WIGGLE_P10:
                s10.angle(clamp(P10_B))
            else:
                s10.angle(clamp(P10_B))
        except:
            pass

        target = P10_B
        next_flip = time.time() + random.uniform(WIGGLE_HOLD_MIN, WIGGLE_HOLD_MAX)

        while not stop_evt.is_set():
            now = time.time()

            # wiggle P10
            if WIGGLE_P10 and now >= next_flip:
                target = P10_A if target == P10_B else P10_B
                next_flip = now + random.uniform(WIGGLE_HOLD_MIN, WIGGLE_HOLD_MAX)

            try:
                s8.angle(clamp(LOCK_P8))
                s9.angle(clamp(LOCK_P9))
                s10.angle(clamp(target))
            except:
                pass

            time.sleep(WIGGLE_INTERVAL)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return stop_evt, t


# ===================== MAIN =====================

def main():
    print("=== Pidog Camera Avoid (Full) ===")

    # Step 1) pose chuẩn từ file config (mượt)
    cfg = load_pose_config(POSE_FILE)
    apply_pose_config(cfg)

    # Step 2) boot pidog từ Matthew class
    print("[STEP2] Boot Pidog by MatthewPidogBootClass...")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)

    # start camera web
    print("[CAM] start web stream...")
    camera_start_web()
    print("[CAM] web stream enabled. (Try open browser to Pi IP, typical port 8000/9000)")

    # start head lock during loop
    head_stop, head_thread = start_head_lock()

    # log file
    log_f, log_w = open_log_file(LOG_FILE)
    log_event(log_w, "START", detail="begin loop")
    log_f.flush()

    try:
        # đảm bảo đứng ổn
        dog.do_action("stand", speed=SPEED_STAND)
        dog.wait_all_done()
        sleep(0.3)

        while True:
            # ---- IMU bump detect first (khẩn cấp) ----
            if imu_bump_detect(dog):
                turn_dir = "right"  # va chạm -> ưu tiên rẽ phải cho nhất quán
                print("\n[BUMP] IMU bump detected -> stand hold -> backup 5s -> turn right")
                log_event(log_w, "BUMP", detail="imu bump", turn_dir=turn_dir)
                log_f.flush()

                try:
                    dog.rgb_strip.set_mode("bark", "red", bps=3)
                except:
                    pass

                # đứng yên 0.5s trước khi lùi (giảm té)
                dog.do_action("stand", speed=SPEED_STAND)
                dog.wait_all_done()
                sleep(0.5)

                do_backward_for_seconds(dog, seconds=BACKUP_SEC_BUMP, speed=SPEED_BACK)
                do_turn(dog, turn_dir, steps=TURN_STEPS_BUMP, speed=SPEED_TURN)
                continue

            # ---- camera obstacle detect ----
            n, x, w, h, area = camera_read_color()
            if n > 0 and area >= AREA_SEEN:
                # quyết định rẽ trái/phải
                turn_dir = decide_turn_dir_from_x(x) if USE_X_TO_DECIDE else TURN_PREFERENCE

                reason = ""
                if USE_X_TO_DECIDE and x is not None:
                    reason = f"x={x} center={CENTER_X} => turn {turn_dir} (avoid opposite side)"
                else:
                    reason = f"fixed preference => turn {turn_dir}"

                print(f"\n[OBS] n={n} x={x} w={w} h={h} area={area} | {reason}")
                log_event(log_w, "OBSTACLE", detail=reason, color=(n, x, w, h, area), turn_dir=turn_dir)
                log_f.flush()

                # stand yên 2s
                try:
                    dog.rgb_strip.set_mode("boom", "blue", bps=3)
                except:
                    pass

                if hasattr(dog, "body_stop"):
                    dog.body_stop()
                sleep(0.05)

                dog.do_action("stand", speed=SPEED_STAND)
                dog.wait_all_done()

                print(f"[OBS] hold stand {HOLD_STAND_SEC:.1f}s ...")
                time.sleep(HOLD_STAND_SEC)

                # quá gần -> bark 3
                if area >= AREA_CLOSE:
                    print("[OBS] too close -> bark x3")
                    log_event(log_w, "BARK", detail="too close", color=(n, x, w, h, area), turn_dir=turn_dir)
                    log_f.flush()
                    safe_bark_3(dog)

                # lùi + rẽ
                print("[OBS] backup then turn...")
                do_backward_for_seconds(dog, seconds=BACKUP_SEC_OBSTACLE, speed=SPEED_BACK)
                do_turn(dog, turn_dir, steps=TURN_STEPS_OBSTACLE, speed=SPEED_TURN)

                try:
                    dog.rgb_strip.set_mode("breath", "white", bps=0.6)
                except:
                    pass
                continue

            # ---- safe: forward normally ----
            try:
                dog.rgb_strip.set_mode("breath", "white", bps=0.6)
            except:
                pass

            dog.do_action("forward", step_count=1, speed=SPEED_FORWARD)
            dog.wait_all_done()
            sleep(0.02)

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt")
        log_event(log_w, "STOP", detail="keyboard interrupt")
        log_f.flush()

    except Exception as e:
        print(f"\n\033[31mERROR: {e}\033[m")
        log_event(log_w, "ERROR", detail=str(e))
        log_f.flush()

    finally:
        # stop head lock
        if head_stop is not None:
            head_stop.set()
        if head_thread is not None:
            head_thread.join(timeout=0.5)

        # Step 4) return to config pose
        print("[STEP4] Return to config pose then exit.")
        try:
            cfg = load_pose_config(POSE_FILE)
            apply_pose_config(cfg)
        except:
            pass

        try:
            Vilib.camera_close()
        except:
            pass

        try:
            dog.close()
        except:
            pass

        try:
            log_f.close()
        except:
            pass


if __name__ == "__main__":
    main()
