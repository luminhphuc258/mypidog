#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

STEP_DELAY = 0.01      # chuyển mượt
MOVE_STEPS = 25        # số bước nội suy


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos, pose_from, pose_to):
    for s in range(1, MOVE_STEPS + 1):
        t = s / MOVE_STEPS
        for p in PORTS:
            v = pose_from[p] + (pose_to[p] - pose_from[p]) * t
            servos[p].angle(clamp(v))
        sleep(STEP_DELAY)


def main():
    servos = {p: Servo(p) for p in PORTS}

    # ----- load pose đứng chuẩn từ file -----
    base = json.loads(POSE_FILE.read_text())
    for k in base:
        base[k] = clamp(base[k])

    print("Loaded base pose:", base)
    apply_pose(servos, base)
    sleep(0.4)

    # ====== 4 STEP MỚI ======

    # Step 1 – đứng 4 chân thẳng (giống base, chỉnh rất nhẹ nếu muốn)
    STEP1 = {
        "P0": -3,   "P1": 89,  "P2": 9,
        "P3": -80,  "P4": 3,   "P5": 90,
        "P6": 10,   "P7": -90, "P8": -29,
        "P9": 90,   "P10": -90, "P11": 0,
    }

    # Step 2 – chân trước phải bước lên (P2 = +30)
    STEP2 = {
        "P0": -3,   "P1": 89,  "P2": 30,
        "P3": -80,  "P4": 3,   "P5": 90,
        "P6": 10,   "P7": -90, "P8": -29,
        "P9": 90,   "P10": -90, "P11": 0,
    }

    # Step 3 – chân sau phải lùi về sau (P6 = -23, P2 vẫn +30) – hình bạn vừa chụp
    STEP3 = {
        "P0": -3,   "P1": 89,  "P2": 30,
        "P3": -80,  "P4": 3,   "P5": 90,
        "P6": -23,  "P7": -90, "P8": -29,
        "P9": 90,   "P10": -90, "P11": 0,
    }

    # Step 4 – chân trái trước & sau bước (dựa pose hình 4–5)
    STEP4 = {
        "P0": -12,  "P1": 90,  "P2": -7,
        "P3": -90,  "P4": 3,   "P5": 90,
        "P6": -23,  "P7": -90, "P8": -29,
        "P9": 90,   "P10": -90, "P11": 0,
    }

    STEPS = [STEP1, STEP2, STEP3, STEP4]

    # Về STEP1 trước
    move_pose(servos, base, STEP1)
    current = STEP1

    print("Start 4-step amble…")

    while True:
        for nxt in STEPS[1:] + [STEP1]:
            move_pose(servos, current, nxt)
            current = nxt
            sleep(0.005)


if __name__ == "__main__":
    main()
