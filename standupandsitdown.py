#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# === Dùng file config ở THƯ MỤC HIỆN TẠI (current working directory) ===
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"

# ===== chỉnh nhanh =====
REPS = 3

RESET_HOLD_SEC = 0.8
STAND_HOLD_SEC = 0.8
SIT_HOLD_SEC = 0.8

MOVE_STEPS = 30
STEP_DELAY = 0.02

LEG_DELTA = 70  # đổi góc 70 độ

# Motor được phép di chuyển tạm thời: chỉ motor 5 và 7  => P4, P6
MOVE_LEG_INDEXES = [7, 8]

# Hướng từng chân P0..P7: +1 bình thường, -1 đảo chiều
# Motor 3 và 7 ngược => P2 và P6 = -1
LEG_DIR = [
    1,   # P0 (motor 1)
    1,   # P1 (motor 2)
   -1,   # P2 (motor 3)  (reversed)
    1,   # P3 (motor 4)
    1,   # P4 (motor 5)
    1,   # P5 (motor 6)
   -1,   # P6 (motor 7)  (reversed)
    1    # P7 (motor 8)
]

# ===== Head using channel 10 (P10) =====
HEAD_PORT = "P10"
HEAD_SWING = 25
HEAD_HOLD_SEC = 0.35

CLAMP_LO, CLAMP_HI = -90, 90
PORTS = [f"P{i}" for i in range(12)]  # P0..P11


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def load_pose_file(path: Path) -> dict:
    """
    Đọc file pose dạng JSON {'P0':0,...,'P11':0} ở thư mục hiện tại.
    Nếu không có file -> báo lỗi (để anh biết phải chạy tool chỉnh servo trước).
    """
    if not path.exists():
        raise FileNotFoundError(f"Không thấy file pose: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Pose file phải là JSON object kiểu {'P0':0,...,'P11':0}")
    out = {}
    for p in PORTS:
        out[p] = clamp(data.get(p, 0))
    return out


def make_stand_selective(sit: dict) -> dict:
    """
    Chỉ move P4 và P6 (motor 5 và 7); các chân khác giữ nguyên.
    Hướng từng chân lấy từ LEG_DIR.
    """
    stand = dict(sit)
    for i in range(8):
        p = f"P{i}"
        if i in MOVE_LEG_INDEXES:
            stand[p] = clamp(sit[p] + LEG_DIR[i] * LEG_DELTA)
        else:
            stand[p] = sit[p]
    return stand


def lerp(a, b, t: float):
    return a + (b - a) * t


def apply_pose(servos: dict, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def move_pose(servos: dict, pose_from: dict, pose_to: dict, steps=MOVE_STEPS, step_delay=STEP_DELAY):
    for s in range(1, steps + 1):
        t = s / steps
        for p in PORTS:
            v = clamp(lerp(pose_from[p], pose_to[p], t))
            servos[p].angle(v)
        sleep(step_delay)


def head_swing(servos: dict, base_pose: dict, port=HEAD_PORT, swing=HEAD_SWING):
    """
    Lắc đầu nhẹ quanh góc trong pose.
    """
    base = clamp(base_pose[port])
    left = clamp(base - swing)
    right = clamp(base + swing)

    servos[port].angle(base);  sleep(0.12)
    servos[port].angle(left);  sleep(HEAD_HOLD_SEC)
    servos[port].angle(base);  sleep(0.12)
    servos[port].angle(right); sleep(HEAD_HOLD_SEC)
    servos[port].angle(base);  sleep(0.12)


def legs_list(pose: dict):
    return [pose[f"P{i}"] for i in range(8)]


def main():
    servos = {p: Servo(p) for p in PORTS}

    # ==== BƯỚC 1: LOAD FILE CONFIG + ĐƯA ROBOT VỀ ĐÚNG VỊ TRÍ ====
    sit_pose = load_pose_file(POSE_FILE)
    stand_pose = make_stand_selective(sit_pose)

    print("Loaded SIT pose from:", POSE_FILE)
    print("Move motors: 5,7 => ports:", [f"P{i}" for i in MOVE_LEG_INDEXES])
    print("Reversed motors: 3,7 => ports: P2, P6")
    print("LEG_DELTA:", LEG_DELTA)
    print("SIT legs  (P0..P7):", legs_list(sit_pose))
    print("STAND legs(P0..P7):", legs_list(stand_pose))
    print("HEAD baseline", HEAD_PORT, "=", sit_pose[HEAD_PORT], "| swing ±", HEAD_SWING)
    print()

    # Đưa tất cả servo về pose trong file config (tư thế ngồi mới)
    apply_pose(servos, sit_pose)
    sleep(RESET_HOLD_SEC)

    # ==== BƯỚC 2: ĐỨNG LÊN / NGỒI XUỐNG / LẮC ĐẦU NHẸ ====
    for _ in range(REPS):
        # từ ngồi -> đứng (chỉ P4, P6 khác nên chỉ 2 chân sau chạy)
        move_pose(servos, sit_pose, stand_pose)
        sleep(0.2)
        head_swing(servos, stand_pose)   # lắc đầu khi đang đứng
        sleep(STAND_HOLD_SEC)

        # từ đứng -> ngồi lại
        move_pose(servos, stand_pose, sit_pose)
        sleep(0.2)
        head_swing(servos, sit_pose)     # lắc đầu khi đang ngồi
        sleep(SIT_HOLD_SEC)

    # cuối cùng trả về tư thế ngồi gốc
    apply_pose(servos, sit_pose)
    sleep(0.3)


if __name__ == "__main__":
    main()
