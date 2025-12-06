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

# ===== KHÓA CHÂN TRỤ KHI A->B =====
LOCK_PORTS_A_TO_B = {"P0", "P1", "P2", "P3"}

# ===== TUNE SPEED Ở ĐÂY =====
MOVE_STEPS   = 25
FRAME_DELAY  = 0.03
SETTLE_SEC   = 1.0


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def lerp(a, b, t):
    return a + (b - a) * t


class SmoothPoseRunner:
    def __init__(self, order):
        self.order = order
        self.servos = {p: Servo(p) for p in order}
        # lưu “góc hiện tại” nội bộ (không đọc từ hardware)
        self.current = {p: 0.0 for p in order}

    def go_to(self, target: dict, steps=MOVE_STEPS, frame_delay=FRAME_DELAY, settle_sec=SETTLE_SEC,
              freeze_ports=None):
        """
        freeze_ports: set các port không được di chuyển trong lần transition này
        """
        freeze_ports = set(freeze_ports or [])

        for i in range(1, steps + 1):
            t = i / steps
            for port in self.order:
                if port in freeze_ports:
                    continue
                if port not in target:
                    continue

                a0 = self.current.get(port, 0.0)
                a1 = float(target[port])
                ang = clamp(lerp(a0, a1, t))
                self.servos[port].angle(ang)

            sleep(frame_delay)

        # update current (trừ ports bị freeze thì giữ nguyên)
        for port in self.order:
            if port in freeze_ports:
                continue
            if port in target:
                self.current[port] = float(target[port])

        if settle_sec and settle_sec > 0:
            print(f"[STABLE] settle {settle_sec:.1f}s ...")
            time.sleep(settle_sec)


def main():
    print("=== robot_hat ONLY: PREPOSE -> delay -> POSE_FROM_IMAGE (freeze P0-P3 on A->B) ===")

    runner = SmoothPoseRunner(ORDER_SAFE)

    # Step 1: về pose A bình thường (cho phép move hết để vào đúng trạng thái prepose)
    print("[1] PREPOSE (smooth, move ALL)...")
    runner.go_to(PREPOSE, steps=MOVE_STEPS, frame_delay=FRAME_DELAY, settle_sec=1.0, freeze_ports=None)

    # Step 2: delay 1s
    print("[2] Delay 1s ...")
    time.sleep(1.0)

    # Step 3: A -> B nhưng KHÓA P0,P1,P2,P3
    print("[3] A -> B (smooth, FREEZE P0-P3)...")
    runner.go_to(POSE_FROM_IMAGE, steps=MOVE_STEPS, frame_delay=FRAME_DELAY,
                 settle_sec=1.0, freeze_ports=LOCK_PORTS_A_TO_B)

    print("[DONE] Finished (no pidog).")


if __name__ == "__main__":
    main()
