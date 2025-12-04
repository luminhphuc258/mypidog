#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# === File pose NGỒI ở thư mục hiện tại ===
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

# ===== tham số chuyển động =====
REPS = 3

RESET_HOLD_SEC = 0.8
STAND_HOLD_SEC = 0.8
SIT_HOLD_SEC = 0.8

MOVE_STEPS = 25
STEP_DELAY = 0.02

CLAMP_LO, CLAMP_HI = -90, 90
PORTS = [f"P{i}" for i in range(12)]  # P0..P11

# ===== TƯ THẾ ĐỨNG CHUẨN (theo hình anh gửi) =====
STAND_TEMPLATE = {
    "P0":  32,
    "P1":  39,
    "P2": -14,
    "P3": -33,
    "P4":   8,
    "P5":  65,
    "P6":  15,
    "P7": -74,
    # đầu và đuôi nếu muốn dùng luôn theo hình:
    "P8": -29,   # head yaw
    "P9":  90,   # head roll
    "P10": -90,  # head pitch
    "P11":  0,   # tail
}

# ===== Head (có thể dùng lại để lắc nhẹ) =====
HEAD_PORT = "P8"   # anh có thể đổi nếu muốn
HEAD_SWING = 15
HEAD_HOLD_SEC = 0.25


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def load_pose_file(path: Path) -> dict:
    """Load tư thế NGỒI từ file JSON {'P0':..,'P11':..}."""
    if not path.exists():
        raise FileNotFoundError(f"Không thấy file pose: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Pose file phải là JSON object kiểu {'P0':0,...,'P11':0}")
    out = {}
    for p in PORTS:
        out[p] = clamp(data.get(p, 0))
    return out


def make_stand_pose_from_sit(sit: dict) -> dict:
    """
    Tạo tư thế ĐỨNG từ tư thế NGỒI:
    - Lấy sit_pose làm base.
    - Ghi đè lại P0..P7 (và P8..P11 nếu có trong template) theo STAND_TEMPLATE.
    """
    stand = dict(sit)
    for k, v in STAND_TEMPLATE.items():
        stand[k] = clamp(v)
    return stand


def apply_pose(servos: dict, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_legs_group(servos: dict,
                    pose_from: dict,
                    pose_to: dict,
                    leg_indices,
                    steps=MOVE_STEPS,
                    step_delay=STEP_DELAY):
    """
    Chỉ nội suy và cập nhật các chân trong leg_indices.
    Các servo khác giữ nguyên vị trí hiện tại.
    """
    for s in range(1, steps + 1):
        t = s / steps
        for i in leg_indices:
            p = f"P{i}"
            a0 = pose_from[p]
            a1 = pose_to[p]
            angle = clamp(a0 + (a1 - a0) * t)
            servos[p].angle(angle)
        sleep(step_delay)


def head_swing(servos: dict, base_pose: dict,
               port=HEAD_PORT, swing=HEAD_SWING):
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
    # khởi tạo servo
    servos = {p: Servo(p) for p in PORTS}

    # load tư thế NGỒI từ file
    sit_pose = load_pose_file(POSE_FILE)
    # tạo tư thế ĐỨNG theo template
    stand_pose = make_stand_pose_from_sit(sit_pose)

    print("Loaded SIT pose from:", POSE_FILE)
    print("SIT legs  (P0..P7):", legs_list(sit_pose))
    print("STAND legs(P0..P7):", legs_list(stand_pose))
    print()

    # Đưa robot về tư thế NGỒI chuẩn
    apply_pose(servos, sit_pose)
    sleep(RESET_HOLD_SEC)

    front_legs = [0, 1, 2, 3]
    rear_legs  = [4, 5, 6, 7]

    for _ in range(REPS):
        # ===== NGỒI -> ĐỨNG =====
        # 1) chỉnh chân trước trước
        move_legs_group(servos, sit_pose, stand_pose, front_legs)
        sleep(0.1)
        # 2) sau đó mới chỉnh chân sau
        move_legs_group(servos, sit_pose, stand_pose, rear_legs)
        sleep(0.2)
        head_swing(servos, stand_pose)
        sleep(STAND_HOLD_SEC)

        # ===== ĐỨNG -> NGỒI ===== (làm ngược lại: chân sau rồi chân trước)
        move_legs_group(servos, stand_pose, sit_pose, rear_legs)
        sleep(0.1)
        move_legs_group(servos, stand_pose, sit_pose, front_legs)
        sleep(0.2)
        head_swing(servos, sit_pose)
        sleep(SIT_HOLD_SEC)

    # Trả về tư thế ngồi
    apply_pose(servos, sit_pose)
    sleep(0.3)


if __name__ == "__main__":
    main()
