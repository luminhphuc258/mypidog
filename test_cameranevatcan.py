#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pidog_avoid_camera_imu.py

Flow:
STEP 1) Apply pose "chuẩn" từ pidog_pose_config.txt bằng robot_hat.Servo (mượt, không dùng pidog)
STEP 2) Boot/init Pidog bằng MatthewPidogBootClass (file matthewpidogclassinit.py của bạn)
STEP 3) Camera (Vilib) phát hiện vật cản (color detection) + IMU phát hiện va chạm:
        - Bình thường: đi thẳng từng bước nhỏ
        - Nếu thấy vật cản: né sang hướng ít vật cản hơn (scan trái/phải bằng head yaw P8)
        - Nếu vật cản quá gần: bark 3 tiếng rồi né mạnh hơn
        - Nếu phát hiện va chạm (IMU spike): stop -> lùi 5s -> rẽ phải -> trot
STEP 4) Close pidog, rồi đưa robot về pose file config lần cuối và thoát

Ghi chú:
- "Obstacle by camera" ở đây dùng Color Detection của Vilib (nên dán/tạo marker màu đỏ trên vật cản).
"""

import json
import time
import random
import threading
import subprocess
from pathlib import Path
from time import sleep

from robot_hat import Servo
from vilib import Vilib
from pidog.preset_actions import bark

from matthewpidogclassinit import MatthewPidogBootClass  # bạn đã đổi tên class/module như đã nói

# ===================== USER CONFIG =====================

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
SERVO_PORTS = [f"P{i}" for i in range(12)]  # P0..P11

# Camera detection mode (Vilib color detection)
DETECT_COLOR = "red"  # red, green, blue, yellow, orange, purple

# Thresholds camera (tune theo nhà bạn)
# - area = w*h của "largest color block"
AREA_SEEN = 1200          # có thấy vật cản (marker) không
AREA_CLOSE = 14000        # quá gần -> bark 3 tiếng + né mạnh

# Head scan (P8 = head yaw)
SCAN_LEFT = -40
SCAN_RIGHT = 40
SCAN_CENTER_FALLBACK = 32   # nếu config file không có P8 thì giữ 32

# Motion tuning (tune để đỡ té)
SPEED_FORWARD = 210
SPEED_TURN = 220
SPEED_TROT = 160
SPEED_STAND = 2
SPEED_SIT = 20

STEP_DELAY_BETWEEN_SERVO = 0.012  # càng lớn càng "êm" khi apply config pose
SETTLE_SEC = 1.0

# IMU collision detect (tune)
IMU_HZ = 50
JERK_SUM_THRESHOLD = 9000     # tổng |Δax|+|Δay|+|Δaz| vượt ngưỡng => coi là va chạm
COOLDOWN_AFTER_HIT = 2.0      # sau va chạm, bỏ qua detect thêm vài giây

# Backup behavior on collision
BACKUP_SECONDS = 5.0

# ===================== HELPERS =====================

ANGLE_MIN, ANGLE_MAX = -90, 90


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


def cleanup_gpio_busy():
    # Giúp giảm lỗi "GPIO busy" khi trước đó có process khác giữ GPIO
    subprocess.run(
        ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
        check=False
    )
    sleep(0.2)


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


def apply_pose_config(cfg: dict, step_delay=STEP_DELAY_BETWEEN_SERVO, settle_sec=SETTLE_SEC):
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

    # set lần lượt để mượt
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


def safe_bark_3(dog):
    try:
        for _ in range(3):
            bark(dog, [0, 0, 0])
            dog.wait_all_done()
            sleep(0.15)
    except Exception:
        # fallback nếu preset_actions.bark khác version
        try:
            for _ in range(3):
                dog.do_action("bark", speed=80)
                dog.wait_all_done()
                sleep(0.15)
        except Exception as e:
            print(f"[WARN] Bark failed: {e}")


# ===================== CAMERA (Vilib) =====================

def camera_start():
    # web=False để nhẹ, bạn muốn xem thì đổi web=True
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)
    Vilib.color_detect(color=DETECT_COLOR)
    sleep(0.4)
    print("[CAM] started + color_detect =", DETECT_COLOR)


def camera_read_color():
    """
    return (n, x, w, h, area)
    """
    try:
        n = int(Vilib.detect_obj_parameter.get("color_n", 0))
        if n <= 0:
            return 0, 0, 0, 0, 0
        x = int(Vilib.detect_obj_parameter.get("color_x", 0))
        w = int(Vilib.detect_obj_parameter.get("color_w", 0))
        h = int(Vilib.detect_obj_parameter.get("color_h", 0))
        area = w * h
        return n, x, w, h, area
    except Exception:
        return 0, 0, 0, 0, 0


def scan_left_right_area(p8_center: int, dwell=0.18):
    """
    Dùng robot_hat.Servo("P8") quay đầu sang trái/phải để đo area.
    Trả: (left_area, right_area)
    """
    try:
        s8 = Servo("P8")
    except Exception as e:
        print(f"[WARN] cannot init Servo(P8) for scan: {e}")
        return 0, 0

    # left
    try:
        s8.angle(clamp(SCAN_LEFT))
        sleep(dwell)
        _, _, _, _, left_area = camera_read_color()
    except Exception:
        left_area = 0

    # right
    try:
        s8.angle(clamp(SCAN_RIGHT))
        sleep(dwell)
        _, _, _, _, right_area = camera_read_color()
    except Exception:
        right_area = 0

    # back center
    try:
        s8.angle(clamp(p8_center))
        sleep(0.05)
    except Exception:
        pass

    return left_area, right_area


# ===================== IMU COLLISION THREAD =====================

def start_imu_collision_monitor(dog):
    """
    Monitor accData jerk. Nếu jerk lớn -> set hit_event.
    accData docs: dog.accData (ax, ay, az) (raw). :contentReference[oaicite:2]{index=2}
    """
    hit_event = threading.Event()
    stop_event = threading.Event()

    def worker():
        last = None
        last_hit_time = 0.0
        period = 1.0 / float(IMU_HZ)

        while not stop_event.is_set():
            try:
                ax, ay, az = dog.accData  # raw ints
                now = time.time()

                if last is not None:
                    lax, lay, laz = last
                    jerk = abs(ax - lax) + abs(ay - lay) + abs(az - laz)

                    # cooldown sau hit để tránh trigger liên tục
                    if jerk >= JERK_SUM_THRESHOLD and (now - last_hit_time) > COOLDOWN_AFTER_HIT:
                        last_hit_time = now
                        hit_event.set()
                last = (ax, ay, az)
            except Exception:
                pass

            time.sleep(period)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return hit_event, stop_event, t


# ===================== MAIN NAV LOGIC =====================

def do_backward_for_seconds(dog, seconds, speed=180):
    t0 = time.time()
    while time.time() - t0 < seconds:
        dog.do_action("backward", step_count=1, speed=speed)
        dog.wait_all_done()
        sleep(0.02)


def turn_in_direction(dog, direction: str, steps=2, speed=SPEED_TURN):
    if direction == "right":
        dog.do_action("turn_right", step_count=steps, speed=speed)
    else:
        dog.do_action("turn_left", step_count=steps, speed=speed)
    dog.wait_all_done()


def main():
    print("=== Pidog: camera + IMU obstacle avoid ===")

    cleanup_gpio_busy()

    # STEP 1) apply config pose FIRST (robot_hat only)
    cfg = load_pose_config(POSE_FILE)
    apply_pose_config(cfg, step_delay=STEP_DELAY_BETWEEN_SERVO, settle_sec=1.0)

    # center head yaw from cfg if available
    p8_center = int(cfg.get("P8", SCAN_CENTER_FALLBACK))

    # STEP 2) boot/init pidog by Matthew class
    print("[STEP2] Boot Pidog by MatthewPidogBootClass...")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    sleep(1.0)  # ổn định thêm để đỡ té

    # STEP 2.5) start camera
    camera_start()

    # start IMU collision monitor
    hit_event, imu_stop, imu_thread = start_imu_collision_monitor(dog)

    try:
        dog.rgb_strip.set_mode("breath", "white", bps=0.6)

        # đảm bảo đứng trước khi đi
        dog.do_action("stand", speed=SPEED_STAND)
        dog.wait_all_done()
        sleep(0.25)

        print("[RUN] walking around... (CTRL+C to stop)")
        while True:
            # nếu va chạm -> lùi 5s, rẽ phải, trot
            if hit_event.is_set():
                hit_event.clear()
                print("\n[HIT] collision detected -> backup 5s -> turn_right -> trot")

                dog.rgb_strip.set_mode("boom", "red", bps=4)
                if hasattr(dog, "body_stop"):
                    dog.body_stop()
                sleep(0.05)

                do_backward_for_seconds(dog, BACKUP_SECONDS, speed=180)

                dog.rgb_strip.set_mode("boom", "yellow", bps=3)
                turn_in_direction(dog, "right", steps=4, speed=SPEED_TURN)

                dog.rgb_strip.set_mode("breath", "white", bps=0.7)
                dog.do_action("trot", step_count=6, speed=SPEED_TROT)
                dog.wait_all_done()
                continue

            # đọc camera
            n, x, w, h, area = camera_read_color()

            if n > 0 and area >= AREA_SEEN:
                # có vật cản (marker color)
                print(f"\n[OBS] n={n} x={x} w={w} h={h} area={area}")

                dog.rgb_strip.set_mode("boom", "blue", bps=3)
                if hasattr(dog, "body_stop"):
                    dog.body_stop()
                sleep(0.05)

                # quá gần -> bark 3 lần
                if area >= AREA_CLOSE:
                    dog.rgb_strip.set_mode("bark", "red", bps=3)
                    safe_bark_3(dog)

                # scan trái/phải để chọn hướng ít vật cản hơn
                left_area, right_area = scan_left_right_area(p8_center=p8_center, dwell=0.18)
                print(f"[SCAN] left_area={left_area} right_area={right_area}")

                # chọn hướng: area nhỏ hơn => ít vật cản
                if left_area == right_area:
                    direction = random.choice(["left", "right"])
                else:
                    direction = "left" if left_area < right_area else "right"

                # né mạnh hơn nếu quá gần
                steps = 4 if area >= AREA_CLOSE else 2
                turn_in_direction(dog, direction, steps=steps, speed=SPEED_TURN)

                dog.rgb_strip.set_mode("breath", "white", bps=0.7)
                continue

            # không thấy vật cản -> đi thẳng
            dog.rgb_strip.set_mode("breath", "white", bps=0.6)
            dog.do_action("forward", step_count=1, speed=SPEED_FORWARD)
            dog.wait_all_done()
            sleep(0.02)

    except KeyboardInterrupt:
        print("\n[STOP] KeyboardInterrupt")
    finally:
        # stop imu thread
        try:
            imu_stop.set()
            imu_thread.join(timeout=0.5)
        except Exception:
            pass

        # close camera
        try:
            Vilib.camera_close()
        except Exception:
            pass

        # STEP 4) close pidog FIRST then return to config pose
        try:
            dog.rgb_strip.set_mode("breath", "white", bps=0.3)
        except Exception:
            pass

        try:
            dog.close()
        except Exception:
            pass

        print("[STEP4] Return to config pose then exit.")
        try:
            apply_pose_config(cfg, step_delay=STEP_DELAY_BETWEEN_SERVO, settle_sec=1.0)
        except Exception as e:
            print("[WARN] apply_pose_config at end failed:", e)


if __name__ == "__main__":
    main()
