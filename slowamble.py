#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# =================== CONFIG ===================
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# Nội suy mềm (sit → stand)
STAND_STEPS = 35
STAND_DELAY = 0.02

# Nội suy cho gait (mạnh, nhanh)
WALK_STEPS = 18
WALK_DELAY = 0.015

CYCLES = 50   # số vòng lặp gait 8 bước


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except:
        x = 0
    return max(lo, min(hi, x))


def lerp(a, b, t):
    return a + (b - a) * t


def load_pose(path: Path) -> dict:
    data = json.loads(path.read_text())
    out = {}
    for p in PORTS:
        out[p] = clamp(data.get(p, 0))
    return out


def apply_pose(servos, pose):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos, pose_from, pose_to, steps, delay):
    for s in range(1, steps + 1):
        t = s / steps
        for p in PORTS:
            v = clamp(lerp(pose_from[p], pose_to[p], t))
            servos[p].angle(v)
        sleep(delay)


# =================== 8 AMBLE STEPS ===================

STEP1 = {
  "P0": -27, "P1": 83, "P2": 43, "P3": -82,
  "P4": 18,  "P5": 86, "P6": -10, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP2 = {
  "P0": -27, "P1": 83, "P2": -8, "P3": -82,
  "P4": 18,  "P5": 86, "P6": -10, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP3 = {
  "P0": -25, "P1": 83, "P2": -8, "P3": -82,
  "P4": 18,  "P5": 86, "P6": 32, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP4 = {
  "P0": -25, "P1": 83, "P2": 62, "P3": -82,
  "P4": 18,  "P5": 86, "P6": 32, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP5 = {
  "P0": 33, "P1": 83, "P2": 62, "P3": -82,
  "P4": 18, "P5": 86, "P6": 32, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP6 = {
  "P0": 34, "P1": 83, "P2": 50, "P3": -82,
  "P4": -14, "P5": 86, "P6": 32, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP7 = {
  "P0": -15, "P1": 65, "P2": 48, "P3": -82,
  "P4": -14, "P5": 86, "P6": 32, "P7": -77,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEP8 = {
  "P0": -41, "P1": 65, "P2": 67, "P3": -82,
  "P4": 55,  "P5": 81, "P6": -38, "P7": -79,
  "P8": -29, "P9": 90, "P10": -90, "P11": 0
}

STEPS = [STEP1, STEP2, STEP3, STEP4, STEP5, STEP6, STEP7, STEP8]


# =================== MAIN GAIT FUNCTION ===================

def gait_loop(servos, sit_pose):
    """
    1. setup bằng pose config
    2. đứng lên theo step1
    3. loop 8 bước mãi mãi
    """

    # --- 1) đưa về pose trong config ---
    apply_pose(servos, sit_pose)
    sleep(0.6)

    # --- 2) từ config → đứng lên (step1) ---
    move_pose(servos, sit_pose, STEPS[0],
              steps=STAND_STEPS, delay=STAND_DELAY)
    current = STEPS[0]
    sleep(0.1)

    # --- 3) loop 8 bước (không quay lại config) ---
    for _ in range(CYCLES):
        for i in range(len(STEPS)):
            nxt = STEPS[(i + 1) % len(STEPS)]
            move_pose(servos, current, nxt,
                      steps=WALK_STEPS, delay=WALK_DELAY)
            current = nxt


def main():
    servos = {p: Servo(p) for p in PORTS}

    sit_pose = load_pose(POSE_FILE)
    gait_loop(servos, sit_pose)

    # kết thúc: giữ nguyên không trả về sit trừ khi bạn muốn
    # apply_pose(servos, sit_pose)


if __name__ == "__main__":
    main()
