#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass

# ===== REAR LEGS (P5, P7) =====
P5_START = 18       # P5 từ +18°
P7_START = -13      # P7 từ -13°
P5_TARGET = 4       # target như trước
P7_TARGET = -1

# ===== LOAD LEGS (LOCK) =====
P4_LOCK = 80        # P4 chịu lực
P6_LOCK = -70       # P6 chịu lực

DELAY = 0.05
ANGLE_MIN, ANGLE_MAX = -90, 90


def clamp(v):
    return max(ANGLE_MIN, min(ANGLE_MAX, int(v)))


def apply_angle(servo, angle):
    try:
        servo.angle(clamp(angle))
    except Exception as e:
        print(f"[WARN] servo error: {e}")


def main():
    print("=== STEP 0: Init servos P4, P5, P6, P7 ===")
    # chỉ cần 4 servo này vì mình đã bỏ bước 4,5
    s4 = Servo("P4")
    s5 = Servo("P5")
    s6 = Servo("P6")
    s7 = Servo("P7")

    # STEP 1: đưa P4, P6 về góc lock
    print("=== STEP 1: Set load legs P4,P6 lock ===")
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    print(f"INIT -> P4 = {P4_LOCK}°, P6 = {P6_LOCK}°")
    time.sleep(0.5)

    # STEP 2: đưa P5, P7 về góc bắt đầu
    print("=== STEP 2: Move P5 to +18°, P7 to -13° ===")
    curr_P5 = P5_START
    curr_P7 = P7_START
    apply_angle(s5, curr_P5)
    apply_angle(s7, curr_P7)
    print(f"INIT -> P5 = {curr_P5}°, P7 = {curr_P7}°")

    # đảm bảo P4, P6 vẫn đang lock
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    time.sleep(1.0)

    # STEP 3: luân phiên move P5, P7 tới target (rear legs)
    print("=== STEP 3: Alternating move REAR legs P5,P7 with P4,P6 locked ===")
    print(f"TARGET -> P5 = {P5_TARGET}°, P7 = {P7_TARGET}°")

    step_idx = 0
    while curr_P5 != P5_TARGET or curr_P7 != P7_TARGET:
        step_idx += 1

        # luôn giữ P4, P6 khóa mỗi vòng lặp
        apply_angle(s4, P4_LOCK)
        apply_angle(s6, P6_LOCK)

        # ---- Move P5 ----
        if curr_P5 != P5_TARGET:
            if P5_TARGET > curr_P5:
                curr_P5 += 1
            elif P5_TARGET < curr_P5:
                curr_P5 -= 1
            apply_angle(s5, curr_P5)
            # lock lại P4, P6
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[REAR {step_idx}] P5 -> {curr_P5}°   "
                  f"(P7 = {curr_P7}° | P4 = {P4_LOCK}° | P6 = {P6_LOCK}°)")
            time.sleep(DELAY)

        # ---- Move P7 ----
        if curr_P7 != P7_TARGET:
            if P7_TARGET > curr_P7:
                curr_P7 += 1
            elif P7_TARGET < curr_P7:
                curr_P7 -= 1
            apply_angle(s7, curr_P7)
            # lock lại P4, P6
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[REAR {step_idx}] P7 -> {curr_P7}°   "
                  f"(P5 = {curr_P5}° | P4 = {P4_LOCK}° | P6 = {P6_LOCK}°)")
            time.sleep(DELAY)

    print("=== REAR LEGS DONE ===")
    print(f"FINAL REAR -> P5 = {curr_P5}°, P7 = {curr_P7}°")
    print(f"P4 (lock) = {P4_LOCK}°, P6 (lock) = {P6_LOCK}°")

    # === BOOT PIDOG + STAND ===
    print("=== STEP 4: Boot MatthewPidog and stand ===")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)

    print("[PIDOG] do_action('stand') ...")
    dog.do_action("stand", speed=30)
    dog.wait_all_done()
    time.sleep(0.5)
    print("[OK] Pidog stand pose applied. Done.")


if __name__ == "__main__":
    main()
