#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass

POSE_FILE = "./pidog_pose_config.txt"

ANGLE_MIN, ANGLE_MAX = -90, 90

def clamp(v, lo=ANGLE_MIN, hi=ANGLE_MAX):
    v = int(v)
    return max(lo, min(hi, v))


# ==============================================================
# 1) LOAD POSE CHUẨN TỪ FILE → ĐƯA ROBOT VỀ SAFE-POSE
# ==============================================================

def load_safe_pose_from_config(file_path):
    print("[STEP 1] Loading baseline pose BEFORE boot Pidog...")

    # Load JSON pose file
    with open(file_path, "r") as f:
        pose_data = json.load(f)

    # Apply servo angles
    for servo_name, angle in pose_data.items():
        try:
            s = Servo(servo_name)
            s.angle(clamp(angle))
        except Exception as e:
            print(f"   [WARN] Cannot move {servo_name}: {e}")

    time.sleep(1)
    print("[OK] Robot is now in SAFE baseline pose.")


# ==============================================================
# 2) XOAY ĐẦU – P8/P9 CỐ ĐỊNH – P10 LẮC TRÁI/PHẢI
# ==============================================================

def head_wiggle(p8=32, p9=-90, p10_min=-60, p10_max=60):

    print("[STEP 4] Starting head wiggle... press Ctrl+C to stop.")

    s8 = Servo("P8")
    s9 = Servo("P9")
    s10 = Servo("P10")

    # Fix P8, P9
    s8.angle(clamp(p8))
    s9.angle(clamp(p9))
    s10.angle(0)
    time.sleep(0.4)

    direction = 1
    cur = p10_min
    STEP = 3
    DELAY = 0.06

    try:
        while True:
            s8.angle(clamp(p8))
            s9.angle(clamp(p9))
            s10.angle(clamp(cur))

            time.sleep(DELAY)
            cur += STEP * direction

            if cur >= p10_max:
                cur = p10_max
                direction = -1
            elif cur <= p10_min:
                cur = p10_min
                direction = 1

    except KeyboardInterrupt:
        print("\n[STOP] Returning head to center...")
        s10.angle(0)
        time.sleep(0.3)


# ==============================================================
# MAIN FLOW
# ==============================================================

if __name__ == "__main__":

    print("\n=========== MATTHEW PIDOG – SAFE FLOW START ===========\n")

    # STEP 1: Load safe pose from config BEFORE boot
    load_safe_pose_from_config(POSE_FILE)

    # STEP 2: Khởi tạo Pidog (MatthewPidogBootClass)
    print("[STEP 2] Booting MatthewPidogBootClass...")
    dog = MatthewPidogBootClass()
    dog.boot()   # Important boot sequence
    time.sleep(0.5)

    # STEP 3: Cho robot ĐỨNG LÊN chuẩn Pidog
    print("[STEP 3] Command robot to STAND...")
    dog.do_action("stand", speed=1)
    dog.wait_all_done()
    time.sleep(0.3)

    # STEP 4: Bắt đầu xoay đầu
    head_wiggle()
