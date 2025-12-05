#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# tăng tốc: ít bước nội suy hơn, delay nhỏ hơn
MOVE_STEPS = 18
STEP_DELAY = 0.008


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos, pose_from: dict, pose_to: dict):
    for s in range(1, MOVE_STEPS + 1):
        t = s / MOVE_STEPS
        for p in PORTS:
            v = pose_from[p] + (pose_to[p] - pose_from[p]) * t
            servos[p].angle(clamp(v))
        sleep(STEP_DELAY)


def main():
    servos = {p: Servo(p) for p in PORTS}

    # --- Load pose đứng chuẩn từ file (để sync servo) ---
    base = json.loads(POSE_FILE.read_text())
    for k in base:
        base[k] = clamp(base[k])

    print("Loaded base pose from config:", base)
    apply_pose(servos, base)
    sleep(0.3)

    # ============ 4 STEP MỚI ============

    # Step 1 – đứng 4 chân thẳng
    STEP1 = {
        "P0": -3,   "P1": 89,  "P2": 9,
        "P3": -80,  "P4": 3,   "P5": 90,
        "P6": 10,   "P7": -90, "P8": -53,   # head yaw chỉnh -53
        "P9": 90,   "P10": -90, "P11": 0,
    }

    # Step 2 – chân trước phải bước lên, P3/P5/P6 mạnh hơn
    STEP2 = {
        "P0": -3,   "P1": 89,  "P2": 30,   # FR lên
        "P3": -71,  "P4": 0,   "P5": 89,   # lực bên trái sau
        "P6": 14,   "P7": -90,
        "P8": -53,  "P9": 90,  "P10": -90, "P11": 0,
    }

    # Step 3 – cân lại, hai bên làm việc, P4/P6 đã chỉnh mới
    STEP3 = {
        "P0": -4,   "P1": 89,  "P2": -15,
        "P3": -80,  "P4": 12,  "P5": 90,
        "P6": -32,  "P7": -77,
        "P8": -53,  "P9": 90,  "P10": -90, "P11": 0,
    }

    # Step 4 – chân trái trước & sau bước, P4 mạnh hơn (lùi nhiều)
    STEP4 = {
        "P0": -41,  "P1": 89,  "P2": 30,
        "P3": -56,  "P4": -19, "P5": 90,
        "P6": -23,  "P7": -90,
        "P8": -53,  "P9": 90,  "P10": -90, "P11": 0,
    }

    STEPS = [STEP1, STEP2, STEP3, STEP4]

    # về STEP1 trước cho khớp dáng đứng
    move_pose(servos, base, STEP1)
    current = STEP1

    print("Start 4-step fast amble…")

    while True:
        for nxt in STEPS[1:] + [STEP1]:
            move_pose(servos, current, nxt)
            current = nxt
            # nghỉ rất nhỏ giữa các bước để không bị giật
            sleep(0.003)


if __name__ == "__main__":
    main()
