#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# === File pose ở THƯ MỤC HIỆN TẠI ===
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

# ===== tham số chuyển động =====
REPS = 3

RESET_HOLD_SEC = 0.8
STAND_HOLD_SEC = 0.8
SIT_HOLD_SEC = 0.8

MOVE_STEPS = 25       # bước nội suy
STEP_DELAY = 0.02

# 🔧 QUAN TRỌNG: 2 giá trị này quyết định tư thế ĐỨNG
#   - DELTA_P5: cộng vào góc của P5 (motor 6)
#   - DELTA_P7: cộng vào góc của P7 (motor 8)
# GỢI Ý BAN ĐẦU: thử -40 cho cả hai (hoặc -30 nếu thấy mạnh quá).
DELTA_P5 = 40      # thử: nếu chân quay ra sau, đổi thành +40
DELTA_P7 = -40      # thử: nếu chân quay ra sau, đổi thành +40

# ===== Head using channel 10 (P10) =====
HEAD_PORT = "P10"
HEAD_SWING = 20
HEAD_HOLD_SEC = 0.3

CLAMP_LO, CLAMP_HI = -90, 90
PORTS = [f"P{i}" for i in range(12)]  # P0..P11


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def load_pose_file(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Không thấy file pose: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Pose file phải là JSON object kiểu {'P0':0,...,'P11':0}")
    out = {}
    for p in PORTS:
        out[p] = clamp(data.get(p, 0))
    return out


def make_stand_from_sit(sit: dict) -> dict:
    """
    Tạo tư thế đứng từ tư thế ngồi.
    Chỉ chỉnh P5 (motor 6) & P7 (motor 8),
    các chân khác giữ nguyên giá trị trong file config.
    """
    stand = dict(sit)

    # motor 6 -> P5
    stand["P5"] = clamp(sit["P5"] + DELTA_P5)

    # motor 8 -> P7
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


def legs_list(pose: dict):
    return [pose[f"P{i}"] for i in range(8)]


def main():
    servos = {p: Servo(p) for p in PORTS}

    sit_pose = load_pose_file(POSE_FILE)
    stand_pose = make_stand_from_sit(sit_pose)

    print("Loaded SIT pose from:", POSE_FILE)
    print("P5 (motor 6) sit/stand:", sit_pose["P5"], "->", stand_pose["P5"],
          "| DELTA_P5 =", DELTA_P5)
    print("P7 (motor 8) sit/stand:", sit_pose["P7"], "->", stand_pose["P7"],
          "| DELTA_P7 =", DELTA_P7)
    print("SIT legs  (P0..P7):", legs_list(sit_pose))
    print("STAND legs(P0..P7):", legs_list(stand_pose))
    print()

    # Đưa về tư thế ngồi chuẩn từ file
    apply_pose(servos, sit_pose)
    sleep(RESET_HOLD_SEC)

    for _ in range(REPS):
        # Ngồi -> đứng
        move_pose(servos, sit_pose, stand_pose)
        sleep(0.2)
        head_swing(servos, stand_pose)
        sleep(STAND_HOLD_SEC)

        # Đứng -> ngồi
        move_pose(servos, stand_pose, sit_pose)
        sleep(0.2)
        head_swing(servos, sit_pose)
        sleep(SIT_HOLD_SEC)

    apply_pose(servos, sit_pose)
    sleep(0.3)


if __name__ == "__main__":
    main()
