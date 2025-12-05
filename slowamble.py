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
MOVE_STEPS = 25        # nội suy mượt


# ====== UTILS ======
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


# ==========================
# STEP 1 – ĐỨNG THẲNG 4 CHÂN
# (pose base trong file config)
# ==========================
STEP1 = {
    "P0": -3,   "P1": 89,  "P2": 9,
    "P3": -80,  "P4": 3,   "P5": 90,
    "P6": 10,   "P7": -90, "P8": -29,
    "P9": 90,   "P10": -90, "P11": 0,
}

# ==========================
# STEP 2 – CHÂN TRƯỚC PHẢI BƯỚC LÊN
# (P2 = +30)
# ==========================
STEP2 = {
    "P0": -3,   "P1": 89,  "P2": 30,
    "P3": -80,  "P4": 3,   "P5": 90,
    "P6": 10,   "P7": -90, "P8": -29,
    "P9": 90,   "P10": -90, "P11": 0,
}

# ==========================
# STEP 3 – CHÂN SAU PHẢI ĐẨY RA SAU
# (P6 = -23, FR vẫn +30)
# ==========================
STEP3 = {
    "P0": -3,   "P1": 89,  "P2": 30,
    "P3": -80,  "P4": 3,   "P5": 90,
    "P6": -23,  "P7": -90, "P8": -29,
    "P9": 90,   "P10": -90, "P11": 0,
}

# ==========================
# STEP 4 – CHÂN TRƯỚC TRÁI DỊCH CHUYỂN
# (P0 = -22, P4 = +10, P6 = +6)
# ==========================
STEP4 = {
    "P0": -22,  "P1": 90,  "P2": 30,
    "P3": -80,  "P4": 10,  "P5": 90,
    "P6": 6,    "P7": -90, "P8": -29,
    "P9": 90,   "P10": -90, "P11": 0,
}

# ==========================
# STEP 5 – CHÂN SAU TRÁI ĐẨY
# (P4 = +35, P5 = +85, P6 = -20, P7 = -84)
# ==========================
STEP5 = {
    "P0": -22,  "P1": 90,  "P2": 30,
    "P3": -80,  "P4": 35,  "P5": 85,
    "P6": -20,  "P7": -84, "P8": -29,
    "P9": 90,   "P10": -90, "P11": 0,
}

STEPS = [STEP1, STEP2, STEP3, STEP4, STEP5]


# ==========================
# MAIN
# ==========================
def main():
    servos = {p: Servo(p) for p in PORTS}

    # Load base pose từ file (để chắc chắn servo sync với STEP1)
    if POSE_FILE.exists():
        base_file = json.loads(POSE_FILE.read_text())
        for k in base_file:
            base_file[k] = clamp(base_file[k])
        print("Loaded base pose from config:", base_file)
    else:
        base_file = STEP1

    # Về pose đứng trong file rồi mượt sang STEP1
    apply_pose(servos, base_file)
    sleep(0.4)
    move_pose(servos, base_file, STEP1)
    sleep(0.1)

    print("Start 5-step amble gait…")

    current = STEP1
    while True:
        for nxt in STEPS[1:] + [STEP1]:
            move_pose(servos, current, nxt)
            current = nxt
            sleep(0.005)   # giữ nhỏ để không té


if __name__ == "__main__":
    main()
