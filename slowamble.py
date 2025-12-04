#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# Nội suy chung (ngồi/đứng)
MOVE_STEPS = 35
STEP_DELAY = 0.02

# Nội suy cho bước đi – ÍT bước hơn => quay nhanh, “mạnh” hơn
WALK_STEPS = 15
WALK_DELAY = 0.015

CYCLES = 5

DELTA_P5 = 90   # motor 6
DELTA_P7 = -90  # motor 8


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def lerp(a, b, t: float):
    return a + (b - a) * t


def load_pose_file(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Không thấy file pose: {path}")
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


# ===== 7 STEP SLOW AMBLE =====
STEP1 = {
  "P0": -27, "P1": 79, "P2": 43, "P3": -56,
  "P4": 48,  "P5": 67, "P6": -45, "P7": -3,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP2 = {
  "P0": -27, "P1": 82, "P2": 43, "P3": -85,
  "P4": 48,  "P5": 36, "P6": -45, "P7": -32,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP3 = {
  "P0": 43, "P1": 79, "P2": 49, "P3": -90,
  "P4": 48, "P5": 90, "P6": -45, "P7": -36,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP4 = {
  "P0": 31, "P1": 79, "P2": 25, "P3": -90,
  "P4": 66, "P5": 50, "P6": -60, "P7": -25,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP5 = {
  "P0": -34, "P1": 79, "P2": 25, "P3": -90,
  "P4": -14, "P5": 90, "P6": -60, "P7": -25,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP6 = {
  "P0": -23, "P1": 84, "P2": 17, "P3": -56,
  "P4": 32,  "P5": 50, "P6": -35, "P7": -55,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEP7 = {
  "P0": -23, "P1": 84, "P2": 2,  "P3": -56,
  "P4": 32,  "P5": 50, "P6": -35, "P7": -90,
  "P8": -22, "P9": 90, "P10": -90, "P11": 0
}

STEPS = [STEP1, STEP2, STEP3, STEP4, STEP5, STEP6, STEP7]


def slow_amble_cycle(servos: dict, sit_pose: dict, stand_pose: dict):
    # 1. về tư thế trong config
    apply_pose(servos, sit_pose)
    sleep(0.5)

    # 2. sit -> stand (êm)
    move_pose(servos, sit_pose, stand_pose,
              steps=MOVE_STEPS, step_delay=STEP_DELAY)
    sleep(0.2)

    # 3. stand -> 7 bước slow amble (NHANH HƠN)
    current = stand_pose
    for pose in STEPS:
        move_pose(servos, current, pose,
                  steps=WALK_STEPS, step_delay=WALK_DELAY)
        current = pose

    # 4. step7 -> ngồi lại (êm)
    move_pose(servos, current, sit_pose,
              steps=MOVE_STEPS, step_delay=STEP_DELAY)
    sleep(0.4)


def main():
    servos = {p: Servo(p) for p in PORTS}

    sit_pose = load_pose_file(POSE_FILE)
    stand_pose = make_stand_from_sit(sit_pose)

    for _ in range(CYCLES):
        slow_amble_cycle(servos, sit_pose, stand_pose)

    apply_pose(servos, sit_pose)
    sleep(0.5)


if __name__ == "__main__":
    main()
