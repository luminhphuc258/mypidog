#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass

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

# ✅ FIX: P1 bị ngược chiều -> đảo dấu (mirror)
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

    print("=== SERVOS DONE -> BOOT PIDOG & STAND ===")

    # STEP 6: boot pidog + stand
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)

    dog.do_action("stand", speed=30)
    dog.wait_all_done()
    time.sleep(0.5)

    print("=== DONE ===")


if __name__ == "__main__":
    main()
