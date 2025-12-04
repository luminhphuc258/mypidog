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

# Chỉ cho 2 motor sau hoạt động: motor 6 & 8  => P5, P7
# Mapping motor ↔ P:
#   motor 6 -> P5 -> index 5
#   motor 8 -> P7 -> index 7
MOVE_LEG_INDEXES = [5, 7]

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
    Tạo tư thế đứng từ tư thế ngồi.

    Theo test:
      - +20 = quay lên trời
      - -20 = quay xuống đất

    Nhưng thực tế:
      - Motor 6 (P5) => dùng -LEG_DELTA thì quay xuống đất (OK)
      - Motor 8 (P7) => dùng -LEG_DELTA lại quay lên trời (ngược)
        => P7 phải dùng +LEG_DELTA để quay xuống đất.

    => Xử lý riêng:
        - i == 5 (P5 / motor 6): sit[p] - LEG_DELTA
        - i == 7 (P7 / motor 8): sit[p] + LEG_DELTA
    """
    stand = dict(sit)
    for i in range(8):  # P0..P7
        p = f"P{i}"
        if i == 5:  # motor 6, P5: -LEG_DELTA => xuống đất
            stand[p] = clamp(sit[p] - LEG_DELTA)
        elif i == 7:  # motor 8, P7: +LEG_DELTA => xuống đất (do bị đảo)
            stand[p] = clamp(sit[p] + LEG_DELTA)
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

    # ==== BƯỚC 1: LOAD FILE CONFIG + ĐƯA ROBOT VỀ ĐÚNG VỊ TRÍ (NGỒI) ====
    sit_pose = load_pose_file(POSE_FILE)
    stand_pose = make_stand_selective(sit_pose)

    print("Loaded SIT pose from:", POSE_FILE)
    print("MOVE_LEG_INDEXES (index P0..P7):", MOVE_LEG_INDEXES)
    print("=> Move motors (1-based):", [i + 1 for i in MOVE_LEG_INDEXES])   # nên ra [6, 8]
    print("LEG_DELTA:", LEG_DELTA)
    print("SIT legs  (P0..P7):", legs_list(sit_pose))
    print("STAND legs(P0..P7):", legs_list(stand_pose))
    print("HEAD baseline", HEAD_PORT, "=", sit_pose[HEAD_PORT], "| swing ±", HEAD_SWING)
    print()

    # Đưa tất cả servo về pose trong file config (tư thế ngồi)
    apply_pose(servos, sit_pose)
    sleep(RESET_HOLD_SEC)

    # ==== BƯỚC 2: ĐỨNG LÊN / NGỒI XUỐNG / LẮC ĐẦU NHẸ ====
    for _ in range(REPS):
        # từ ngồi -> đứng
        move_pose(servos, sit_pose, stand_pose)
        sleep(0.2)
        head_swing(servos, stand_pose)
        sleep(STAND_HOLD_SEC)

        # từ đứng -> ngồi lại
        move_pose(servos, stand_pose, sit_pose)
        sleep(0.2)
        head_swing(servos, sit_pose)
        sleep(SIT_HOLD_SEC)

    # cuối cùng trả về tư thế ngồi gốc
    apply_pose(servos, sit_pose)
    sleep(0.3)


if __name__ == "__main__":
    main()
