#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from time import sleep

from robot_hat import Servo
from pidog.preset_actions import push_up
from matthewpidogclassinit import MatthewPidogBootClass

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
SERVO_PORTS = [f"P{i}" for i in range(12)]  # P0..P11
DELAY_BETWEEN_WRITES = 0.01  # giống code của bạn -> mượt
SETTLE_SEC = 1.0             # delay sau mỗi phase

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
    """
    Load JSON pose from file.
    Expected format: {"P0": 0, ..., "P11": 0}
    Missing keys will default to 0.
    """
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


def main():
    print("=== 4-STEP FLOW ===")

    # Step 1) set robot to pose from file (robot_hat only)
    cfg = load_pose_config(POSE_FILE)
    apply_pose_config(cfg,step_delay=DELAY_BETWEEN_WRITES, settle_sec=1.0)

    # Step 2) boot/init pidog via Matthew class
    print("[STEP2] Boot Pidog by MatthewPidogBootClass...")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)  # cho servo ổn định thêm để đỡ té

    try:
        # Step 3) actions: push_up -> sit -> stand
        print("[STEP3] Actions: push_up -> sit -> stand")
        dog.rgb_strip.set_mode("breath", "white", bps=0.6)

        push_up(dog, speed=20)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("sit", speed=20)
        dog.wait_all_done()
        time.sleep(0.3)

        dog.do_action("stand", speed=10)
        dog.wait_all_done()
        time.sleep(0.3)

    finally:
        # Step 4) return to pose from file (robot_hat only) then close
        print("[STEP4] Return to config pose then exit.")
      
        apply_pose_config(cfg, step_delay=DELAY_BETWEEN_WRITES, settle_sec=1.0)

if __name__ == "__main__":
    main()
