#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from robot_hat import Servo

# Góc bắt đầu
P5_START = 18      # P5 từ +18°
P7_START = -13     # P7 từ -13°

# Góc đích
P5_TARGET = 90     # xoay tới +90°
P7_TARGET = -90    # xoay tới -90°

DELAY = 0.05       # thời gian chờ giữa mỗi bước
ANGLE_MIN, ANGLE_MAX = -90, 90


def clamp(v):
    return max(ANGLE_MIN, min(ANGLE_MAX, int(v)))


def apply_angle(servo, angle):
    try:
        servo.angle(clamp(angle))
    except Exception as e:
        print(f"[WARN] servo error: {e}")


def main():
    print("=== STEP 1: Move P5 to +18°, P7 to -13° ===")

    s5 = Servo("P5")
    s7 = Servo("P7")

    curr_P5 = P5_START
    curr_P7 = P7_START

    apply_angle(s5, curr_P5)
    apply_angle(s7, curr_P7)
    print(f"INIT -> P5 = {curr_P5}°, P7 = {curr_P7}°")

    time.sleep(1.0)

    print("=== STEP 2: Move to fixed targets (P5=90, P7=-90) ===")
    print(f"TARGET -> P5 = {P5_TARGET}°, P7 = {P7_TARGET}°")

    step_idx = 0

    while curr_P5 != P5_TARGET or curr_P7 != P7_TARGET:
        step_idx += 1

        # ---- Move P5 ----
        if curr_P5 != P5_TARGET:
            if P5_TARGET > curr_P5:
                curr_P5 += 1
            elif P5_TARGET < curr_P5:
                curr_P5 -= 1
            apply_angle(s5, curr_P5)
            print(f"[STEP {step_idx}] P5 -> {curr_P5}°   (P7 = {curr_P7}°)")
            time.sleep(DELAY)

        # ---- Move P7 ----
        if curr_P7 != P7_TARGET:
            if P7_TARGET > curr_P7:
                curr_P7 += 1
            elif P7_TARGET < curr_P7:
                curr_P7 -= 1
            apply_angle(s7, curr_P7)
            print(f"[STEP {step_idx}] P7 -> {curr_P7}°   (P5 = {curr_P5}°)")
            time.sleep(DELAY)

    print("=== DONE! ===")
    print(f"FINAL -> P5 = {curr_P5}°, P7 = {curr_P7}°")


if __name__ == "__main__":
    main()
