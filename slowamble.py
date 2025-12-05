#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# ============== CẤU HÌNH CHUNG ==============
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# Nội suy mềm từ config -> step1
SETUP_STEPS = 25
SETUP_DELAY = 0.02

# Nội suy cho gait: nhanh nhưng vẫn mượt
MOVE_STEPS = 8        # càng nhỏ → chuyển càng nhanh, lực hơn
STEP_DELAY = 0.008      # delay giữa mỗi bước nội suy
STEP_HOLD = 0.0        # gần như không nghỉ ở mỗi step

CYCLES = 80            # số vòng lặp 4 step


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


def apply_pose(servos: dict, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos: dict, pose_from: dict, pose_to: dict,
              steps: int, step_delay: float):
    """Nội suy từ pose_from -> pose_to."""
    for s in range(1, steps + 1):
        t = s / steps
        for p in PORTS:
            v = clamp(lerp(pose_from[p], pose_to[p], t))
            servos[p].angle(v)
        sleep(step_delay)


# ============== 4 STEP MỚI (từ 4 hình) ==============

# Hình 1
STEP1 = {  # stand
    "P0": 52, "P1": -19, "P2": -49, "P3": 29,
    "P4": 59, "P5": -5,  "P6": -40, "P7": 8,
    "P8": -29, "P9": 90, "P10": -90, "P11": 0,
}

# Hình 2
STEP2 = dict(STEP1)
STEP2["P4"] = STEP1["P4"] - 20   # hông sau trái đưa về phía trước
STEP2["P5"] = STEP1["P5"] + 30   # nhấc bàn chân sau trái lên

# Hình 3
STEP3 = dict(STEP2)
STEP3["P4"] = STEP1["P4"] + 10   # hông sau trái đạp ra sau
STEP3["P5"] = STEP1["P5"] + 5    # hơi duỗi chân, chạm đất chắc

# Hình 4
STEP4 = dict(STEP1)
STEP4["P6"] = STEP1["P6"] + 20   # hông sau phải đưa về phía trước
STEP4["P7"] = STEP1["P7"] - 30   

STEPS = [STEP1, STEP2, STEP3, STEP4]


def gait_4steps(servos: dict, base_pose: dict, cycles: int = CYCLES):
    # 1) load & apply pose từ config
    apply_pose(servos, base_pose)
    sleep(0.5)

    # 2) config -> STEP1 (setup mềm)
    move_pose(servos, base_pose, STEPS[0],
              steps=SETUP_STEPS, step_delay=SETUP_DELAY)
    current = STEPS[0]
    sleep(0.05)

    # 3) loop 4 bước nhanh & mượt
    for _ in range(cycles):
        for i in range(len(STEPS)):
            nxt = STEPS[(i + 1) % len(STEPS)]  # 1→2→3→4→1→…
            move_pose(servos, current, nxt,
                      steps=MOVE_STEPS, step_delay=STEP_DELAY)
            if STEP_HOLD > 0:
                sleep(STEP_HOLD)
            current = nxt


def main():
    servos = {p: Servo(p) for p in PORTS}

    base_pose = load_pose_file(POSE_FILE)
    gait_4steps(servos, base_pose, cycles=CYCLES)

    # nếu muốn khi kết thúc quay lại pose config thì mở dòng sau:
    # apply_pose(servos, base_pose)


if __name__ == "__main__":
    main()
