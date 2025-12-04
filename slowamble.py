#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# ================== CẤU HÌNH CHUNG ==================
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

MOVE_STEPS = 35      # bước nội suy giữa 2 pose
STEP_DELAY = 0.02
CYCLES = 5           # số lần lặp: sit -> stand -> 7 bước -> sit

# chỉnh từ tư thế ngồi trong file sang đứng
DELTA_P5 = 90   # P5 (motor 6)
DELTA_P7 = -90  # P7 (motor 8)


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
    stand["P5"] = clamp(sit["P5"] + DELTA_P5)  # motor 6
    stand["P7"] = clamp(sit["P7"] + DELTA_P7)  # motor 8
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


# ===================================================
# 7 STEP SLOW AMBLE – GÓC BẠN ĐÃ CALIBRATE SẴN
# Motor mapping (để nhớ):
#  - Motor 1  = chân trái trước  (P1)
#  - Motor 3  = chân phải trước (P3)
#  - Motor 5  = chân trái sau   (P5)
#  - Motor 7  = chân phải sau   (P7)
#  - Motor 0,2,4,6 = khớp nối tương ứng
# ===================================================

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
    """
    1. Về tư thế trong config (ngồi)
    2. Đứng dậy
    3. Chạy 7 step slow amble
    4. Quay lại tư thế trong config (ngồi)
    """
    # 1. về tư thế config
    apply_pose(servos, sit_pose)
    sleep(0.5)

    # 2. sit -> stand
    move_pose(servos, sit_pose, stand_pose)
    sleep(0.2)

    # 3. stand -> step1..step7
    current = stand_pose
    for pose in STEPS:
        move_pose(servos, current, pose)
        current = pose

    # 4. step7 -> sit pose
    move_pose(servos, current, sit_pose)
    sleep(0.4)


def main():
    servos = {p: Servo(p) for p in PORTS}

    # load pose trong config (ngồi) và tạo tư thế đứng
    sit_pose = load_pose_file(POSE_FILE)
    stand_pose = make_stand_from_sit(sit_pose)

    for _ in range(CYCLES):
        slow_amble_cycle(servos, sit_pose, stand_pose)

    # kết thúc: giữ ở tư thế config
    apply_pose(servos, sit_pose)
    sleep(0.5)


if __name__ == "__main__":
    main()
