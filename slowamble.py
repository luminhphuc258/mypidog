#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# ==== TĂNG TỐC ĐỘ ====
# trước: MOVE_STEPS = 25, STEP_DELAY = 0.01
MOVE_STEPS = 10       # ít bước nội suy hơn -> servo quay nhanh hơn
STEP_DELAY = 0.004    # delay nhỏ hơn -> chuyển động “mạnh” và nhanh hơn


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

    # --- load pose đứng chuẩn từ file (để sync servo) ---
    base = json.loads(POSE_FILE.read_text())
    for k in base:
        base[k] = clamp(base[k])

    print("Loaded base pose:", base)
    apply_pose(servos, base)
    sleep(0.3)

    # ========= 4 STEP (giữ nguyên góc) =========

    STEP1 = {
        "P0": -3,   "P1": 89,  "P2": 9,
        "P3": -80,  "P4": 5,   "P5": 90,
        "P6": 10,   "P7": -90, "P8": -53,
        "P9": 90,   "P10": -90, "P11": 0,
    }

    STEP2 = {
        "P0": -3,   "P1": 89,  "P2": 30,
        "P3": -71,  "P4": 0,   "P5": 89,
        "P6": 14,   "P7": -90,
        "P8": -53,  "P9": 90,  "P10": -90, "P11": 0,
    }

    STEP3 = {
        "P0": -3,   "P1": 89,  "P2": 30,
        "P3": -80,  "P4": 5,   "P5": 90,
        "P6": -23,  "P7": -90,
        "P8": -53,  "P9": 90,  "P10": -90, "P11": 0,
    }

    STEP4 = {
        "P0": -41,  "P1": 89,  "P2": 30,
        "P3": -56,  "P4": -19, "P5": 90,
        "P6": -23,  "P7": -90,
        "P8": -53,  "P9": 90,  "P10": -90, "P11": 0,
    }

    STEPS = [STEP1, STEP2, STEP3, STEP4]

    # Về STEP1 trước cho đúng dáng
    move_pose(servos, base, STEP1)
    current = STEP1

    print("Start new 4-step amble (faster)…")

    while True:
        for nxt in STEPS[1:] + [STEP1]:
            move_pose(servos, current, nxt)
            current = nxt
            # nghỉ rất nhỏ giữa các bước
            sleep(0.002)


if __name__ == "__main__":
    main()
