#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# ========= FILE =========
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
GAIT_FILE = Path.cwd() / "dangdithang_thuvien.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

FRAME_DELAY = 0.025   # nhỏ = đi nhanh hơn, lớn = đi chậm hơn


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict):
    """Áp góc trực tiếp cho tất cả P0..P11."""
    for p in PORTS:
        servos[p].angle(clamp(pose.get(p, 0)))


def load_base_pose() -> dict:
    data = json.loads(POSE_FILE.read_text())
    base = {k: clamp(v) for k, v in data.items()}
    print("Base pose from config:", base)
    return base


def load_gait_frames():
    raw = GAIT_FILE.read_text()

    # vá JSON nếu cần
    if not raw.lstrip().startswith("["):
        raw = "[\n" + raw
    if not raw.rstrip().endswith("]"):
        raw = raw.rstrip() + "\n]"

    frames = json.loads(raw)
    # đảm bảo mỗi frame đủ 12 cổng
    fixed = []
    for f in frames:
        pose = {}
        for p in PORTS:
            pose[p] = clamp(f.get(p, 0))
        fixed.append(pose)

    print(f"Loaded {len(fixed)} gait frames")
    return fixed


def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) load pose đứng chuẩn + đưa robot về dáng đó
    base = load_base_pose()
    apply_pose(servos, base)
    sleep(0.6)

    # 2) load các frame dáng đi thẳng
    gait_frames = load_gait_frames()

    # Đưa robot từ base -> frame đầu tiên cho mượt
    first = gait_frames[0]
    steps = 30
    for s in range(1, steps + 1):
        t = s / steps
        pose = {}
        for p in PORTS:
            v = base[p] + (first[p] - base[p]) * t
            pose[p] = v
        apply_pose(servos, pose)
        sleep(FRAME_DELAY)

    print("Start gait loop (Ctrl+C để dừng)…")

    # 3) lặp vô hạn theo chuỗi gait
    try:
        while True:
            for i, frame in enumerate(gait_frames):
                apply_pose(servos, frame)
                sleep(FRAME_DELAY)
    except KeyboardInterrupt:
        print("\nStop by user, về lại pose base.")
        apply_pose(servos, base)
        sleep(0.5)


if __name__ == "__main__":
    main()
