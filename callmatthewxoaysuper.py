#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass

# ===== REAR LEGS (P5, P7) =====
P5_START = 18       # P5 từ +18°
P7_START = -13      # P7 từ -13°
P5_TARGET = 4       # target hiện tại (giữ nguyên theo code bạn đang dùng)
P7_TARGET = -1

# ===== LOAD LEGS (LOCK) =====
P4_LOCK = 80        # P4 chịu lực
P6_LOCK = -70       # P6 chịu lực

# ===== FRONT SHOULDERS (P0, P2) =====
P0_TARGET = 40      # P0 -> +40°
P2_TARGET = -26     # P2 -> -26°

# ===== FRONT LEGS (P1, P3) =====
P1_START = -25      # P1 từ -25°
P1_TARGET = -65     # tới -65°
P3_START = 4        # P3 từ +4°
P3_TARGET = -68     # tới -68°

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
    print("=== STEP 0: Init servos P0..P7 ===")
    s0 = Servo("P0")
    s1 = Servo("P1")
    s2 = Servo("P2")
    s3 = Servo("P3")
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

    # STEP 4: move P0, P2 về góc vai trước
    print("=== STEP 4: Move shoulders P0,P2 to fixed angles ===")
    apply_angle(s0, P0_TARGET)
    apply_angle(s2, P2_TARGET)
    print(f"P0 -> {P0_TARGET}°, P2 -> {P2_TARGET}°")
    # giữ P4, P6 lock
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    time.sleep(1.0)

    # STEP 5: move FRONT legs P1,P3 lần lượt như P5,P7
    print("=== STEP 5: Alternating move FRONT legs P1,P3 with P4,P6 locked ===")

    curr_P1 = P1_START
    curr_P3 = P3_START
    apply_angle(s1, curr_P1)
    apply_angle(s3, curr_P3)
    print(f"INIT FRONT -> P1 = {curr_P1}°, P3 = {curr_P3}°")
    # vẫn giữ P4,P6 lock
    apply_angle(s4, P4_LOCK)
    apply_angle(s6, P6_LOCK)
    time.sleep(0.5)

    front_step = 0
    while curr_P1 != P1_TARGET or curr_P3 != P3_TARGET:
        front_step += 1

        # giữ P4,P6 khóa
        apply_angle(s4, P4_LOCK)
        apply_angle(s6, P6_LOCK)

        # ---- Move P1 ----
        if curr_P1 != P1_TARGET:
            if P1_TARGET > curr_P1:
                curr_P1 += 1
            elif P1_TARGET < curr_P1:
                curr_P1 -= 1
            apply_angle(s1, curr_P1)
            # lock lại
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[FRONT {front_step}] P1 -> {curr_P1}°   "
                  f"(P3 = {curr_P3}° | P4 = {P4_LOCK}° | P6 = {P6_LOCK}°)")
            time.sleep(DELAY)

        # ---- Move P3 ----
        if curr_P3 != P3_TARGET:
            if P3_TARGET > curr_P3:
                curr_P3 += 1
            elif P3_TARGET < curr_P3:
                curr_P3 -= 1
            apply_angle(s3, curr_P3)
            # lock lại
            apply_angle(s4, P4_LOCK)
            apply_angle(s6, P6_LOCK)
            print(f"[FRONT {front_step}] P3 -> {curr_P3}°   "
                  f"(P1 = {curr_P1}° | P4 = {P4_LOCK}° | P6 = {P6_LOCK}°)")
            time.sleep(DELAY)

    print("=== FRONT LEGS DONE ===")
    print(f"FINAL FRONT -> P1 = {curr_P1}°, P3 = {curr_P3}°")
    print(f"P0 = {P0_TARGET}°, P2 = {P2_TARGET}°")
    print(f"P4 (lock) = {P4_LOCK}°, P6 (lock) = {P6_LOCK}°")

    # STEP 6: Boot MatthewPidog + stand
    print("=== STEP 6: Boot MatthewPidog and stand ===")
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
