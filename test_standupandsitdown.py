#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from time import sleep
from robot_hat import Servo


# =================== POSE A: PRE-POSE ===================
PREPOSE = {
    "P0":  -4,  "P1":  87, "P2":  18, "P3": -90,
    "P4":  43,  "P5": -22, "P6": -29, "P7":  23,
    "P8":  32,  "P9": -68, "P10": -90, "P11": 0,
}

# =================== POSE B: THEO HÌNH BẠN GỬI ===================
POSE_FROM_IMAGE = {
    "P0":  -3,  "P1":  89, "P2":   9, "P3": -80,
    "P4":   3,  "P5":  90, "P6":  10, "P7": -90,
    "P8": -29,  "P9":  90, "P10": -90, "P11": 0,
}

# Thứ tự set để đỡ té: head/tail -> rear -> front
ORDER_SAFE = ["P8","P9","P10","P11","P4","P5","P6","P7","P0","P1","P2","P3"]


# ===== TUNE SPEED Ở ĐÂY =====
MOVE_STEPS   = 25     # càng nhiều càng mượt (15~35)
FRAME_DELAY  = 0.03   # mỗi frame nghỉ bao lâu (0.02~0.06)
SETTLE_SEC   = 1.0    # nghỉ sau khi xong 1 pose


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def lerp(a, b, t):
    return a + (b - a) * t


def smooth_pose_transition(servos: dict, current: dict, target: dict, order, steps=MOVE_STEPS, frame_delay=FRAME_DELAY):
    # đi theo nhiều bước nhỏ để không giật
    for i in range(1, steps + 1):
        t = i / steps
        for port in order:
            if port not in target:
                continue
            a0 = current.get(port, 0.0)
            a1 = target[port]
            ang = clamp(lerp(a0, a1, t))
            servos[port].angle(ang)
        sleep(frame_delay)


def apply_pose_smooth(target_pose: dict, order=ORDER_SAFE, settle_sec=SETTLE_SEC):
    # tạo servo 1 lần
    servos = {p: Servo(p) for p in order}

    # giả sử current = góc cuối lần trước (nếu chưa có thì dùng 0)
    # -> ở lần đầu, current sẽ là 0, nhưng ta vẫn đi mượt về target
    if not hasattr(apply_pose_smooth, "_current"):
        apply_pose_smooth._current = {p: 0.0 for p in servos.keys()}

    current = apply_pose_smooth._current

    # smooth transition
    smooth_pose_transition(servos, current, target_pose, order)

    # cập nhật “current”
    for p in order:
        if p in target_pose:
            current[p] = float(target_pose[p])

    print(f"[STABLE] settle {settle_sec:.1f}s ...")
    time.sleep(settle_sec)


def main():
    print("=== robot_hat ONLY: PREPOSE -> delay -> POSE_FROM_IMAGE (SLOW/MOOTH) ===")

    print("[1] PREPOSE (smooth)...")
    apply_pose_smooth(PREPOSE, ORDER_SAFE, settle_sec=1.0)

    print("[2] Delay 1s ...")
    time.sleep(1.0)

    print("[3] POSE_FROM_IMAGE (smooth)...")
    apply_pose_smooth(POSE_FROM_IMAGE, ORDER_SAFE, settle_sec=1.0)

    print("[DONE] Finished (no pidog).")


if __name__ == "__main__":
    main()
