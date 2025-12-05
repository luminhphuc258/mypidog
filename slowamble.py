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

    # Base bạn đang dùng (để dễ nhìn):
    # P0=-3, P1=89, P2=9, P3=-80,
    # P4=3, P5=90, P6=10, P7=-90 ...

    print("Loaded base pose from config:", base)
    apply_pose(servos, base)
    sleep(0.5)

    # ----- STEP 1: đứng (gần = base, chỉ chỉnh nhẹ trái) -----
    step1 = dict(base)
    # chỉnh nhẹ FL & RL cho cân (rất nhỏ, không ảnh hưởng hướng)
    step1["P0"] = base["P0"] - 2   # hơi đưa FL về phía trước
    step1["P4"] = base["P4"] + 2   # hơi đưa RL về phía sau

    # ----- STEP 2: FR bước lên, trái chỉnh nhẹ -----
    step2 = dict(step1)
    step2["P2"] = 30               # FR lên trước (như hình bạn chụp)
    # cân bên trái chút xíu để đỡ lệch
    step2["P0"] = step1["P0"] - 3  # FL hơi tiến thêm
    step2["P4"] = step1["P4"] + 3  # RL hơi lui thêm

    # ----- STEP 3: RR đẩy ra sau, trái cũng đẩy NHẸ -----
    step3 = dict(step1)
    step3["P6"] = -23              # RR đẩy sau (như bạn chụp)
    # RL đẩy rất nhẹ cùng hướng nhưng biên độ nhỏ hơn
    step3["P4"] = step1["P4"] + 5

    seq = [step1, step2, step3]

    print("Starting balanced amble walk...")

    current = step1
    # mượt về step1 từ base
    move_pose(servos, base, step1)

    while True:
        for nxt in seq[1:] + [step1]:
            move_pose(servos, current, nxt)
            current = nxt
            sleep(0.005)


if __name__ == "__main__":
    main()
