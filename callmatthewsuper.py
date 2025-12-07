#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import random
import threading
from pathlib import Path
from time import sleep

from robot_hat import Servo
from pidog.preset_actions import push_up
from matthewpidogclassinit import MatthewPidogBootClass
from pidog.preset_actions import push_up, bark
POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
SERVO_PORTS = [f"P{i}" for i in range(12)]  # P0..P11
DELAY_BETWEEN_WRITES = 0.01
SETTLE_SEC = 1.0
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


def apply_pose_config(cfg: dict, step_delay=DELAY_BETWEEN_WRITES, settle_sec=SETTLE_SEC):
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


# ===================== HEAD LOCK + WIGGLE THREAD =====================

# ===================== HEAD LOCK + WIGGLE THREAD =====================

def start_head_controller(
    p8_fixed=32,          # ✅ luôn giữ P8 = 32
    p9_fixed=-90,         # luôn giữ P9
    p10_a=-70, p10_b=-89, # P10 lắc giữa 2 góc
    write_interval=0.10,
    hold_range=(0.5, 1.4)
):
    """
    - P8: luôn giữ p8_fixed
    - P9: luôn giữ p9_fixed
    - P10: thỉnh thoảng đổi giữa p10_a và p10_b
    """
    stop_evt = threading.Event()

    try:
        s8 = Servo("P8")
        s9 = Servo("P9")
        s10 = Servo("P10")
    except Exception as e:
        print(f"[WARN] Cannot init head servos P8/P9/P10: {e}")
        return stop_evt, None

    def worker():
        # set lần đầu
        try:
            s8.angle(clamp(p8_fixed))
            s9.angle(clamp(p9_fixed))
            s10.angle(clamp(p10_b))
        except:
            pass

        target = p10_b
        next_flip = time.time() + random.uniform(*hold_range)

        while not stop_evt.is_set():
            now = time.time()

            if now >= next_flip:
                target = p10_a if target == p10_b else p10_b
                next_flip = now + random.uniform(*hold_range)

            # ✅ ghi đè liên tục để lock P8/P9 + wiggle P10
            try:
                s8.angle(clamp(p8_fixed))
                s9.angle(clamp(p9_fixed))
                s10.angle(clamp(target))
            except:
                pass

            time.sleep(write_interval)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return stop_evt, t



def main():
    print("=== 4-STEP FLOW ===")

    # Step 1) set robot to pose from file (robot_hat only)
    cfg = load_pose_config(POSE_FILE)
    apply_pose_config(cfg, step_delay=DELAY_BETWEEN_WRITES, settle_sec=1.0)

    # Step 2) boot/init pidog via Matthew class
    print("[STEP2] Boot Pidog by MatthewPidogBootClass...")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)  # ổn định thêm

    # Start head controller ONLY during Step 3
    head_stop_evt = None
    head_thread = None

    try:
        # Step 3) actions
        print("[STEP3] Actions: push_up -> sit -> stand (+ head lock/wiggle)")
        dog.rgb_strip.set_mode("breath", "white", bps=0.6)
        bark(dog, [0, 0, -40])
        time.sleep(0.2)

        # bật “khóa P9 + lắc P10”
        head_stop_evt, head_thread = start_head_controller(
           p8_fixed=32,
            p9_fixed=-90,
            p10_a=-70,
            p10_b=-90,
            write_interval=0.08,     # ghi đè khá nhanh để pidog không kéo lệch
            hold_range=(0.6, 1.6)    # “thỉnh thoảng” mới lắc
        )

        push_up(dog, speed=120)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("sit", speed=20)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("stand", speed=1)
        dog.wait_all_done()
        bark(dog, [0, 0, -40])
        time.sleep(0.3)

        dog.do_action("forward", speed=250)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("backward", speed=250)
        dog.wait_all_done()
        bark(dog, [0, 0, -40])
        time.sleep(0.3)

        # turn right 5s
        t0 = time.time()
        while time.time() - t0 < 5.0:
            dog.do_action("turn_right", step_count=1, speed=230)
            dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("stand", speed=1)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("trot", speed=160)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("stand", speed=1)
        dog.wait_all_done()
        time.sleep(0.3)

    finally:
        # tắt head thread trước khi trả pose
        if head_stop_evt is not None:
            head_stop_evt.set()
        if head_thread is not None:
            head_thread.join(timeout=0.5)

        # Step 4) return to pose from file (robot_hat only)
        print("[STEP4] Return to config pose then exit.")


if __name__ == "__main__":
    main()
