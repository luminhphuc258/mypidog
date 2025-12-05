#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from robot_hat import Servo
from pathlib import Path

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
PORTS = [f"P{i}" for i in range(12)]

MOVE_STEPS = 30
STEP_DELAY = 0.01       # nhỏ để robot không bị té


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


def move_pose(servos, old, new):
    for s in range(1, MOVE_STEPS + 1):
        t = s / MOVE_STEPS
        for p in PORTS:
            v = old[p] + (new[p] - old[p]) * t
            servos[p].angle(clamp(v))
        sleep(STEP_DELAY)


def offset(base, **kw):
    out = dict(base)
    for k,v in kw.items():
        out[k] = clamp(out[k] + v)
    return out


def main():
    servos = {p: Servo(p) for p in PORTS}

    BASE = load_pose()
    print("Loaded BASE pose:", BASE)

    apply_pose(servos, BASE)
    sleep(0.3)

    while True:

        # === STEP 1: Nhấc chân trước phải FR (P2,P3) ===
        step1 = offset(BASE,
            P2 = +12,   # shoulder FR nâng ra trước
            P3 = -12    # knee FR gập nhẹ
        )
        move_pose(servos, BASE, step1)

        # === STEP 2: Đặt FR xuống hơi phía trước để tiến ===
        step2 = offset(BASE,
            P2 = -5,   # shoulder kéo FR ra trước
            P3 = +5
        )
        move_pose(servos, step1, step2)

        # === STEP 3: Nhấc chân sau phải RR (P6,P7) ===
        step3 = offset(BASE,
            P6 = +10,   # shoulder RR nâng
            P7 = -10
        )
        move_pose(servos, step2, step3)

        # === STEP 4: Đẩy RR về sau để tạo lực tiến ===
        step4 = offset(BASE,
            P6 = -8,   # shoulder RR đẩy
            P7 = +8
        )
        move_pose(servos, step3, step4)

        # Quay về base cho ổn định
        move_pose(servos, step4, BASE)


if __name__ == "__main__":
    main()
