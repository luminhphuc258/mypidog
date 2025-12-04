#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
from robot_hat import Servo

# ================== THAM SỐ CHUNG ==================
PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

MOVE_STEPS = 35      # số bước nội suy giữa 2 pose
STEP_DELAY = 0.02    # delay giữa mỗi bước nội suy
CYCLES = 5           # đi bao nhiêu vòng slow amble


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    return max(lo, min(hi, int(x)))


def lerp(a, b, t: float):
    return a + (b - a) * t


def apply_pose(servos: dict, pose: dict):
    """Set góc cho tất cả servo theo pose."""
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos: dict, pose_from: dict, pose_to: dict,
              steps=MOVE_STEPS, step_delay=STEP_DELAY):
    """Nội suy mượt từ pose_from -> pose_to."""
    for s in range(1, steps + 1):
        t = s / steps
        for p in PORTS:
            v = clamp(lerp(pose_from[p], pose_to[p], t))
            servos[p].angle(v)
        sleep(step_delay)


# ===================================================
#  MAPPING (để tham khảo)
#  Motor 1  = chân trái trước  -> P1
#  Motor 3  = chân phải trước  -> P3
#  Motor 5  = chân trái sau    -> P5
#  Motor 7  = chân phải sau    -> P7
#  Motor 0,2,4,6 = khớp nối tương ứng
#  Các pose dưới đây đã được bạn calibrate sẵn.
# ===================================================

STEP1 = {
    "P0": -27, "P1": 79, "P2": 43, "P3": -56,
    "P4": 48,  "P5": 67, "P6": -45, "P7": -3,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP2 = {
    "P0": -27, "P1": 82, "P2": 43, "P3": -85,
    "P4": 48,  "P5": 36, "P6": -45, "P7": -32,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP3 = {
    "P0": 43, "P1": 79, "P2": 49, "P3": -90,
    "P4": 48, "P5": 90, "P6": -45, "P7": -36,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP4 = {
    "P0": 31, "P1": 79, "P2": 25, "P3": -90,
    "P4": 66, "P5": 50, "P6": -60, "P7": -25,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP5 = {
    "P0": -34, "P1": 79, "P2": 25, "P3": -90,
    "P4": -14, "P5": 90, "P6": -60, "P7": -25,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP6 = {
    "P0": -23, "P1": 84, "P2": 17, "P3": -56,
    "P4": 32,  "P5": 50, "P6": -35, "P7": -55,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP7 = {
    "P0": -23, "P1": 84, "P2": 2,  "P3": -56,
    "P4": 32,  "P5": 50, "P6": -35, "P7": -90,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP8 = {
    "P0": -15, "P1": 84, "P2": -45, "P3": -58,
    "P4": 40,  "P5": 54, "P6": 1,   "P7": -74,
    "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP9 = {
    "P0": 29, "P1": 84, "P2": 15, "P3": -58,
    "P4": 24, "P5": 65, "P6": -1, "P7": -74,
    "P8": -22,"P9": 90, "P10": -90, "P11": 0
}

STEP10 = {
    "P0": 29, "P1": 84, "P2": 15, "P3": -58,
    "P4": 24, "P5": 65, "P6": -1, "P7": -74,
    "P8": -22,"P9": 90, "P10": -90, "P11": 0
}

STEPS = [STEP1, STEP2, STEP3, STEP4, STEP5,
         STEP6, STEP7, STEP8, STEP9, STEP10]


def slow_amble(servos: dict, cycles=CYCLES):
    """Loop slow amble qua 10 bước, nội suy mượt giữa các pose."""
    # đưa về step1 trước
    apply_pose(servos, STEPS[0])
    sleep(0.5)

    current = STEPS[0]

    for _ in range(cycles):
        for i in range(len(STEPS)):
            nxt = STEPS[(i + 1) % len(STEPS)]
            move_pose(servos, current, nxt)
            current = nxt


def main():
    # Khởi tạo servo
    servos = {p: Servo(p) for p in PORTS}

    # Chạy slow amble
    slow_amble(servos, cycles=CYCLES)

    # Dừng ở pose cuối
    sleep(0.5)


if __name__ == "__main__":
    main()
