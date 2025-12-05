#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
GAIT_FILE = Path.cwd() / "dangdithang_thuvien.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# servo đi nhanh + mượt:
FRAME_DELAY = 0.008      # thời gian nghỉ giữa mỗi bước nội suy
INTERP_STEPS = 6         # số bước nội suy giữa 2 frame (5–8 là khá mượt)


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose.get(p, 0)))


def lerp_pose(pose_from: dict, pose_to: dict, t: float) -> dict:
    """Nội suy 0..1 giữa 2 pose."""
    out = {}
    for p in PORTS:
        a = pose_from.get(p, 0)
        b = pose_to.get(p, 0)
        out[p] = a + (b - a) * t
    return out


def smooth_move(servos, pose_from: dict, pose_to: dict,
                steps: int = INTERP_STEPS, delay: float = FRAME_DELAY):
    """Di chuyển mượt giữa 2 frame."""
    for s in range(1, steps + 1):
        t = s / steps
        pose = lerp_pose(pose_from, pose_to, t)
        apply_pose(servos, pose)
        sleep(delay)


def load_base_pose() -> dict:
    data = json.loads(POSE_FILE.read_text())
    base = {k: clamp(v) for k, v in data.items()}
    print("Base pose from config:", base)
    return base


def load_gait_frames():
    raw = GAIT_FILE.read_text()

    # vá nếu file không có [ ] bọc ngoài
    if not raw.lstrip().startswith("["):
        raw = "[\n" + raw
    if not raw.rstrip().endswith("]"):
        raw = raw.rstrip() + "\n]"

    frames_raw = json.loads(raw)

    frames = []
    for fr in frames_raw:
        pose = {}
        for p in PORTS:
            pose[p] = clamp(fr.get(p, 0))
        frames.append(pose)

    print(f"Loaded {len(frames)} gait frames")
    return frames


def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) Đưa robot về pose chuẩn từ config 1 LẦN ở đầu
    base = load_base_pose()
    apply_pose(servos, base)
    sleep(0.6)

    # 2) Load toàn bộ frame dáng đi thẳng
    gait_frames = load_gait_frames()
    if not gait_frames:
        print("No gait frames found!")
        return

    # Đưa mượt từ base -> frame đầu tiên
    first = gait_frames[0]
    smooth_move(servos, base, first, steps=20, delay=FRAME_DELAY)
    current = first

    print("Start continuous forward gait (Ctrl+C để dừng)…")

    # 3) Loop vô hạn qua tất cả frame (KHÔNG quay về pose base trong loop)
    try:
        while True:
            for i in range(1, len(gait_frames)):
                nxt = gait_frames[i]
                smooth_move(servos, current, nxt)
                current = nxt

            # cuối list nối mượt về frame đầu để chu kỳ khép kín
            nxt = gait_frames[0]
            smooth_move(servos, current, nxt)
            current = nxt
    except KeyboardInterrupt:
        print("\nStop by user – trả robot về pose chuẩn.")
        smooth_move(servos, current, base, steps=20, delay=FRAME_DELAY)
        sleep(0.3)


if __name__ == "__main__":
    main()
