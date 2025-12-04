#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

REPS = 3
RESET_HOLD_SEC = 0.8
STAND_HOLD_SEC = 0.8
SIT_HOLD_SEC = 0.8

MOVE_STEPS = 25
STEP_DELAY = 0.02

DELTA_P5 = 90
DELTA_P7 = -90

HEAD_PORT = "P10"
HEAD_SWING = 20
HEAD_HOLD_SEC = 0.3

CLAMP_LO, CLAMP_HI = -90, 90
PORTS = [f"P{i}" for i in range(12)]


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def load_pose_file(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    out = {}
    for p in PORTS:
        out[p] = clamp(data.get(p, 0))
    return out


def make_stand_from_sit(sit: dict) -> dict:
    stand = dict(sit)
    stand["P5"] = clamp(sit["P5"] + DELTA_P5)
    stand["P7"] = clamp(sit["P7"] + DELTA_P7)
    return stand


def lerp(a, b, t: float):
    return a + (b - a) * t


def apply_pose(servos: dict, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos: dict, pose_from: dict, pose_to: dict,
              steps=MOVE_STEPS, step_delay=STEP_DELAY):
    for s in range(1, steps + 1):
        t = s / steps
        for p in PORTS:
            v = clamp(lerp(pose_from[p], pose_to[p], t))
            servos[p].angle(v)
        sleep(step_delay)


def head_swing(servos: dict, base_pose: dict, port=HEAD_PORT, swing=HEAD_SWING):
    base = clamp(base_pose[port])
    left = clamp(base - swing)
    right = clamp(base + swing)

    servos[port].angle(base);  sleep(0.1)
    servos[port].angle(left);  sleep(HEAD_HOLD_SEC)
    servos[port].angle(base);  sleep(0.1)
    servos[port].angle(right); sleep(HEAD_HOLD_SEC)
    servos[port].angle(base);  sleep(0.1)


# ---------------------------
#  AMBLE WALK 9 STEP
#  Motor mapping:
#   Left Front  = P2
#   Right Front = P6
#   Left Rear   = P4
#   Right Rear  = P8
# ---------------------------

def make_amble_steps(base: dict):
    """
    Tạo 9 steps từ tư thế đứng chuẩn.
    Chỉ thay đổi 4 motor chân: P2, P4, P6, P8
    """

    steps = []

    # Helper tạo copy
    def cp():
        return dict(base)

    # === STEP 1: LF + LR + RR chạm đất, RF đưa lên ===
    s1 = cp()
    s1["P6"] = clamp(base["P6"] - 25)   # RF lift forward
    steps.append(s1)

    # === STEP 2: 4 chân chạm đất ===
    s2 = cp()
    steps.append(s2)

    # === STEP 3: RF đặt xuống, RR đưa lên ===
    s3 = cp()
    s3["P8"] = clamp(base["P8"] - 25)
    steps.append(s3)

    # === STEP 4: 4 chân chạm ===
    s4 = cp()
    steps.append(s4)

    # === STEP 5: RR xuống, LR đưa lên ===
    s5 = cp()
    s5["P4"] = clamp(base["P4"] - 25)
    steps.append(s5)

    # === STEP 6: 4 chân chạm ===
    s6 = cp()
    steps.append(s6)

    # === STEP 7: LR xuống, LF đưa lên ===
    s7 = cp()
    s7["P2"] = clamp(base["P2"] - 25)
    steps.append(s7)

    # === STEP 8: 4 chân chạm ===
    s8 = cp()
    steps.append(s8)

    # === STEP 9: reset lại step 1 nhẹ ===
    s9 = cp()
    s9["P6"] = clamp(base["P6"] - 25)
    steps.append(s9)

    return steps


def amble_walk(servos: dict, stand_pose: dict, loops=3):
    steps = make_amble_steps(stand_pose)

    for _ in range(loops):
        for next_pose in steps:
            move_pose(servos, stand_pose, next_pose, steps=MOVE_STEPS, step_delay=STEP_DELAY)
            sleep(0.05)
            move_pose(servos, next_pose, stand_pose, steps=MOVE_STEPS, step_delay=STEP_DELAY)


def main():
    servos = {p: Servo(p) for p in PORTS}

    sit_pose = load_pose_file(POSE_FILE)
    stand_pose = make_stand_from_sit(sit_pose)

    # Reset -> Sit pose
    apply_pose(servos, sit_pose)
    sleep(RESET_HOLD_SEC)

    # SIT -> STAND
    move_pose(servos, sit_pose, stand_pose)
    sleep(0.3)

    # === RUN AMBLE WALK ===
    print("Starting AMBLE WALK...")
    amble_walk(servos, stand_pose, loops=4)

    # Quay về ngồi
    move_pose(servos, stand_pose, sit_pose)
    sleep(0.2)


if __name__ == "__main__":
    main()
