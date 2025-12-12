#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
SERVO_PORTS = [f"P{i}" for i in range(12)]

# ===== REAR LEGS (P5, P7) =====
P5_START = 18
P7_START = -13
P5_TARGET = 4
P7_TARGET = -1

# ===== LOAD LEGS (LOCK) =====
P4_LOCK = 80
P6_LOCK = -70

# ===== FRONT HIP (P0, P2) =====
P0_TARGET = 40
P2_TARGET = -26

# ===== FRONT KNEE (P1, P3) =====
P1_START = -25
P1_TARGET = -65
P3_START = 4
P3_TARGET = -68

# ✅ P1 ngược chiều -> đảo dấu (mirror)
P1_INVERT = True

DELAY = 0.05
ANGLE_MIN, ANGLE_MAX = -90, 90


def clamp(v):
    return max(ANGLE_MIN, min(ANGLE_MAX, int(v)))


def apply_angle(servo, angle):
    try:
        servo.angle(clamp(angle))
    except Exception as e:
        print(f"[WARN] servo error: {e}")


def apply_angle_p1(servo, angle):
    a = -angle if P1_INVERT else angle
    apply_angle(servo, a)


def load_pose_config(path: Path) -> dict:
    cfg = {p: 0 for p in SERVO_PORTS}
    if not path.exists():
        print(f"[WARN] Pose file not found: {path} -> all zeros.")
        return cfg
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            for k, v in data.items():
                if k in cfg:
                    cfg[k] = clamp(v)
    except Exception as e:
        print(f"[WARN] Pose file parse error: {e} -> all zeros.")
    return cfg


def apply_pose_from_cfg(cfg: dict, per_servo_delay=0.03, settle_sec=1.0):
    print("=== APPLY POSE FROM CONFIG (ALL SERVOS) ===")
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
            apply_angle(servos[p], cfg.get(p, 0))
            time.sleep(per_servo_delay)
        except Exception as e:
            print(f"[WARN] Apply {p} failed: {e}")

    if settle_sec and settle_sec > 0:
        time.sleep(settle_sec)


def main():
    print("=== INIT servos P0..P7 ===")
    s0 = Servo("P0")
    s1 = Servo("P1")
    s2 = Servo("P2")
    s3 = Servo("P3")
    s4 = Servo("P4")
    s5 = Servo("P5")
    s6 = Servo("P6")
    s7 = Servo("P7")

    # STEP 1: lock P4,P6
    print("=== STEP 1: Lock P4,P6 ===")
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    time.sleep(0.5)

    # STEP 2: set P5,P7 start
    print("=== STEP 2: Set P5,P7 start ===")
    curr_P5 = P5_START
    curr_P7 = P7_START
    apply_angle(s5, curr_P5)
    apply_angle(s7, curr_P7)
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    time.sleep(1.0)

    # STEP 3: alternating move P5,P7
    print("=== STEP 3: Move REAR P5,P7 (alt) with P4,P6 locked ===")
    rear_step_idx = 0
    while curr_P5 != P5_TARGET or curr_P7 != P7_TARGET:
        rear_step_idx += 1
        apply_angle(s4, P4_LOCK)
        apply_angle(s6, P6_LOCK)

        if curr_P5 != P5_TARGET:
            curr_P5 += 1 if P5_TARGET > curr_P5 else -1
            apply_angle(s5, curr_P5)
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[REAR {rear_step_idx}] P5 -> {curr_P5}°   (P7={curr_P7}°)")
            time.sleep(DELAY)

        if curr_P7 != P7_TARGET:
            curr_P7 += 1 if P7_TARGET > curr_P7 else -1
            apply_angle(s7, curr_P7)
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[REAR {rear_step_idx}] P7 -> {curr_P7}°   (P5={curr_P5}°)")
            time.sleep(DELAY)

    print("=== REAR DONE ===")

    # STEP 4: set P0,P2
    print("=== STEP 4: Set P0,P2 ===")
    apply_angle(s0, P0_TARGET)
    apply_angle(s2, P2_TARGET)
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    time.sleep(1.0)

    # STEP 5: move P1,P3 alternating (P1 inverted)
    print("=== STEP 5: Move FRONT P1,P3 (alt) with P4,P6 locked ===")
    curr_P1 = P1_START
    curr_P3 = P3_START
    apply_angle_p1(s1, curr_P1)
    apply_angle(s3, curr_P3)

    front_step_idx = 0
    while curr_P1 != P1_TARGET or curr_P3 != P3_TARGET:
        front_step_idx += 1
        apply_angle(s4, P4_LOCK)
        apply_angle(s6, P6_LOCK)

        if curr_P1 != P1_TARGET:
            curr_P1 += 1 if P1_TARGET > curr_P1 else -1
            apply_angle_p1(s1, curr_P1)
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[FRONT {front_step_idx}] P1(cmd) -> {curr_P1}°   (P3={curr_P3}°)")
            time.sleep(DELAY)

        if curr_P3 != P3_TARGET:
            curr_P3 += 1 if P3_TARGET > curr_P3 else -1
            apply_angle(s3, curr_P3)
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[FRONT {front_step_idx}] P3 -> {curr_P3}°   (P1={curr_P1}°)")
            time.sleep(DELAY)

    print("=== 4 LEGS DONE ===")

    # STEP 6: apply pose from file config then end
    cfg = load_pose_config(POSE_FILE)
    apply_pose_from_cfg(cfg, per_servo_delay=0.03, settle_sec=1.0)

    print("=== DONE ===")


if __name__ == "__main__":
    main()
