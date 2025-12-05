#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from robot_hat import Servo
from pathlib import Path


POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
PORTS = [f"P{i}" for i in range(12)]

MOVE_STEPS = 20          # interpolation steps
STEP_DELAY = 0.01        # nhỏ để không té


def clamp(x, lo=-90, hi=90):
    return max(lo, min(hi, int(x)))


def load_pose():
    data = json.loads(POSE_FILE.read_text())
    pose = {}
    for p in PORTS:
        pose[p] = clamp(data.get(p, 0))
    return pose


def apply_pose(servos, pose):
    for p in PORTS:
        servos[p].angle(pose[p])


def move_pose(servos, p_from, p_to):
    for s in range(1, MOVE_STEPS + 1):
        t = s / MOVE_STEPS
        for p in PORTS:
            v = clamp(p_from[p] + (p_to[p] - p_from[p]) * t)
            servos[p].angle(v)
        sleep(STEP_DELAY)


def offset(pose, **changes):
    """ Tạo pose mới bằng cách chỉnh 1 vài servo """
    newp = dict(pose)
    for k, v in changes.items():
        newp[k] = clamp(newp[k] + v)
    return newp


def main():
    servos = {p: Servo(p) for p in PORTS}

    print("Loading base pose...")
    BASE = load_pose()

    # ---- Step 1: Stand base pose ----
    apply_pose(servos, BASE)
    sleep(0.3)

    while True:
        # STEP 2 – Nhấc chân trước phải (FR)
        step2 = offset(BASE,
            P2=+10,   # shoulder FR
            P3=+15    # knee FR
        )
        move_pose(servos, BASE, step2)
        sleep(0.05)

        # STEP 3 – Hạ FR + đẩy nhẹ RL
        step3 = offset(BASE,
            P2=+5,    # nhẹ quay lên tạo lực kéo
            P3=-10,   # duỗi nhẹ
            P4=+5,    # shoulder RL đẩy nhẹ
            P5=+8
        )
        move_pose(servos, step2, step3)
        sleep(0.05)

        # STEP 4 – Nhấc chân sau phải (RR)
        step4 = offset(BASE,
            P6=+12,  # shoulder RR nhấc nhẹ
            P7=+12
        )
        move_pose(servos, step3, step4)
        sleep(0.05)

        # STEP 5 – Đẩy nhẹ RR khi hạ xuống
        step5 = offset(BASE,
            P6=-5,   # push right rear
            P7=+5
        )
        move_pose(servos, step4, step5)
        sleep(0.05)

        # trở về base cho vòng tiếp theo
        move_pose(servos, step5, BASE)
        sleep(0.05)


if __name__ == "__main__":
    main()
