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

# Nội suy giữa các pose (nhanh, ít bước để có lực hơn)
MOVE_STEPS = 10        # càng nhỏ → chuyển càng nhanh
STEP_DELAY = 0.01      # delay giữa mỗi bước nội suy
STEP_HOLD = 0.02       # thời gian “nghỉ” ở mỗi step (rất nhỏ)

CYCLES = 50            # số vòng lặp 4 step


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
              steps=MOVE_STEPS, step_delay=STEP_DELAY):
    """Nội suy nhanh từ pose_from -> pose_to."""
    for s in range(1, steps + 1):
        t = s / steps
        for p in PORTS:
            v = clamp(lerp(pose_from[p], pose_to[p], t))
            servos[p].angle(v)
        sleep(step_delay)


# ============== 4 STEP THEO HÌNH 1–4 ==============

# Hình 1
STEP1 = {
    "P0": -28, "P1": 82, "P2": 43, "P3": -80,
    "P4": 88,  "P5": 2,  "P6": -77, "P7": 8,
    "P8": -29, "P9": 90, "P10": -90, "P11": 0,
}

# Hình 2
STEP2 = {
    "P0": 62, "P1": 64, "P2": 43, "P3": -80,
    "P4": 88, "P5": 2,  "P6": -77, "P7": 8,
    "P8": -29, "P9": 90, "P10": -90, "P11": 0,
}

# Hình 3
STEP3 = {
    "P0": 62, "P1": -8, "P2": -45, "P3": -80,
    "P4": 88, "P5": 2,  "P6": -77, "P7": 8,
    "P8": -29, "P9": 90, "P10": -90, "P11": 0,
}

# Hình 4
STEP4 = {
    "P0": 52, "P1": 15, "P2": -62, "P3": 11,
    "P4": 48, "P5": 18, "P6": -51, "P7": 8,
    "P8": -29, "P9": 90, "P10": -90, "P11": 0,
}

STEPS = [STEP1, STEP2, STEP3, STEP4]


def gait_4steps(servos: dict, start_pose: dict, cycles: int = CYCLES):
    """
    1. robot đã ở pose config (do main() apply trước)
    2. từ pose config → STEP1
    3. loop STEP1 → STEP2 → STEP3 → STEP4 → STEP1 ...
    """

    current = dict(start_pose)

    # pose config -> STEP1
    move_pose(servos, current, STEPS[0])
    sleep(STEP_HOLD)
    current = STEPS[0]

    # loop 4 bước
    for _ in range(cycles):
        for i in range(len(STEPS)):
            nxt = STEPS[(i + 1) % len(STEPS)]  # 1→2→3→4→1→...
            move_pose(servos, current, nxt)
            sleep(STEP_HOLD)
            current = nxt


def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) LOAD & APPLY pose từ file config NGAY KHI BẮT ĐẦU
    base_pose = load_pose_file(POSE_FILE)
    apply_pose(servos, base_pose)
    sleep(0.5)

    # 2) Bắt đầu loop 4 step
    gait_4steps(servos, base_pose, cycles=CYCLES)

    # Nếu muốn kết thúc quay lại pose config thì bỏ comment dòng dưới:
    # apply_pose(servos, base_pose)


if __name__ == "__main__":
    main()
