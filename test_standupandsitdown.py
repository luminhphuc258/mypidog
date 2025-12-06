#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from time import sleep
from robot_hat import Servo


# =================== POSE A: PRE-POSE (bạn có thể chỉnh) ===================
# Nếu bạn muốn prepose khác, chỉ cần sửa mảng này.
PREPOSE = {
    "P0":  -4,
    "P1":  87,
    "P2":  18,
    "P3": -90,
    "P4":  43,
    "P5": -22,
    "P6": -29,
    "P7":  23,
    "P8":  32,
    "P9": -68,
    "P10": -90,
    "P11":  0,
}

# =================== POSE B: THEO HÌNH BẠN GỬI (pidog_pose_config.txt) ===================
POSE_FROM_IMAGE = {
    "P0":  -3,
    "P1":  89,
    "P2":   9,
    "P3": -80,
    "P4":   3,
    "P5":  90,
    "P6":  10,
    "P7": -90,
    "P8": -29,
    "P9":  90,
    "P10": -90,
    "P11":  0,
}

# Thứ tự set để đỡ té: head/tail -> rear -> front
ORDER_SAFE = ["P8", "P9", "P10", "P11", "P4", "P5", "P6", "P7", "P0", "P1", "P2", "P3"]


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def apply_pose(pose: dict, order=ORDER_SAFE, step_delay=0.03):
    for port in order:
        if port not in pose:
            continue
        ang = clamp(float(pose[port]))
        try:
            Servo(port).angle(ang)
            sleep(step_delay)
        except Exception as e:
            print(f"[WARN] {port} -> {ang} fail: {e}")


def main():
    print("=== robot_hat ONLY: PREPOSE -> delay 1s -> POSE_FROM_IMAGE -> DONE ===")

    # Step 1: Pre-pose
    print("[1] Apply PREPOSE ...")
    apply_pose(PREPOSE, ORDER_SAFE, step_delay=0.03)

    # Step 2: delay 1s
    print("[2] Stabilize 1.0s ...")
    time.sleep(1.0)

    # Step 3: set pose theo hình bạn gửi
    print("[3] Apply POSE_FROM_IMAGE ...")
    apply_pose(POSE_FROM_IMAGE, ORDER_SAFE, step_delay=0.03)

    print("[DONE] Finished (no pidog).")


if __name__ == "__main__":
    main()
