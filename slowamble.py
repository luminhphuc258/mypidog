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

# ==========================
# POSE 1 – ĐỨNG 4 CHÂN THẲNG
# ==========================
POSE1 = {
  "P0": -3,  "P1": 89,  "P2": 9,
  "P3": -80, "P4": 11,   "P5": 90,
  "P6": 10,  "P7": -90, "P8": -29,
  "P9": 90,  "P10": -90, "P11": 0
}

# ==============================
# POSE 2 – NHẤC CHÂN TRƯỚC PHẢI
# (P2 tăng lên)
# ==============================
POSE2 = {
  "P0": -3,  "P1": 89,  "P2": 30,
  "P3": -80, "P4": 11,   "P5": 90,
  "P6": 10,  "P7": -90, "P8": -29,
  "P9": 90,  "P10": -90, "P11": 0
}

# ==============================
# POSE 3 – ĐẨY CHÂN SAU PHẢI RA SAU
# (P6 giảm xuống)
# ==============================
POSE3 = {
  "P0": -3,   "P1": 89,  "P2": 9,
  "P3": -80,  "P4": 11,   "P5": 90,
  "P6": -23,  "P7": -90, "P8": -29,
  "P9": 90,   "P10": -90, "P11": 0
}

# ====== UTILS ======
def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except:
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

# ==========================
# MAIN
# ==========================
def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) Load pose đứng chuẩn từ file
    base = json.loads(POSE_FILE.read_text())
    for k in base: base[k] = clamp(base[k])

    print("Loaded base pose from config")
    apply_pose(servos, base)
    sleep(0.5)

    # 2) BẮT ĐẦU ĐI AMBLE
    print("Starting 3-pose amble walk…")

    seq = [POSE1, POSE2, POSE3]

    while True:
        current = base
        for nxt in seq:
            move_pose(servos, current, nxt)
            current = nxt
            sleep(0.01)   # rất nhỏ để không té
        
        # quay về pose đứng trong file config
        move_pose(servos, current, base)
        sleep(0.01)

if __name__ == "__main__":
    main()
