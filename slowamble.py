#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

STEP_DELAY = 0.01
MOVE_STEPS = 25


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

    # ----- BASE từ file (4 chân thẳng) -----
    base = json.loads(POSE_FILE.read_text())
    for k in base:
        base[k] = clamp(base[k])

    print("Loaded base pose from config:", base)
    apply_pose(servos, base)
    sleep(0.5)

    # ----- STEP 1: đứng (chỉnh FL mạnh hơn chút) -----
    step1 = dict(base)
    # trước: base["P0"] - 2  -> giờ cho mạnh hơn
    step1["P0"] = base["P0"] - 6    # FL tiến nhiều hơn
    step1["P4"] = base["P4"] + 2    # RL hơi lui

    # ----- STEP 2: FR bước lên, FL tiến mạnh hơn -----
    step2 = dict(step1)
    step2["P2"] = 30                # FR lên trước như cũ
    # trước: step1["P0"] - 3  -> giờ cho mạnh thêm để kéo sang phải
    step2["P0"] = step1["P0"] - 6   # FL tiến thêm nữa
    step2["P4"] = step1["P4"] + 3   # RL lui nhẹ

    # ----- STEP 3: RR đẩy ra sau, RL đẩy nhẹ -----
    step3 = dict(step1)
    step3["P6"] = -23               # RR đẩy sau
    step3["P4"] = step1["P4"] + 5   # RL hỗ trợ nhẹ

    seq = [step1, step2, step3]

    print("Starting balanced amble walk...")

    current = step1
    move_pose(servos, base, step1)

    while True:
        for nxt in seq[1:] + [step1]:
            move_pose(servos, current, nxt)
            current = nxt
            sleep(0.005)


if __name__ == "__main__":
    main()
