#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

# ====== FILE & CONSTANT ======
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
GAIT_FILE = Path.cwd() / "dangdithang_thuvien.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

# nhanh hơn & mượt
MOVE_STEPS = 12        # nội suy giữa 2 frame
STEP_DELAY = 0.004     # delay rất nhỏ

# chỉ dùng phần ĐI THẲNG, bỏ hết đoạn cuối có ngồi xuống
KEEP_FRAMES = 600      # nếu vẫn dính ngồi, tăng số này lên / nếu đi ít quá, giảm dần

# dùng head / tail chuẩn, không lấy từ file gait
FIXED_HEAD_TAIL = {
    "P8": -29,   # yaw
    "P9":  90,   # roll
    "P10": -90,  # pitch (sẽ lắc từ -90 -> -70)
    "P11":  0,   # tail
}

HEAD_MIN = -90
HEAD_MAX = -70


# ========= UTILS =========
def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict):
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))


def blend_pose(a: dict, b: dict, t: float) -> dict:
    """Nội suy 2 pose a -> b (0..1)."""
    out = {}
    for p in PORTS:
        out[p] = clamp(a[p] + (b[p] - a[p]) * t)
    return out


def move_pose(servos, pose_from: dict, pose_to: dict):
    """Chuyển mượt giữa 2 pose."""
    for s in range(1, MOVE_STEPS + 1):
        t = s / MOVE_STEPS
        pose = blend_pose(pose_from, pose_to, t)
        apply_pose(servos, pose)
        sleep(STEP_DELAY)


# ========= MAIN =========
def main():
    servos = {p: Servo(p) for p in PORTS}

    # --- load pose đứng chuẩn từ file config ---
    base = json.loads(POSE_FILE.read_text())
    for k in base:
        base[k] = clamp(base[k])

    print("Base pose from config:", base)
    apply_pose(servos, base)
    sleep(0.5)

    # --- đọc file gait (dạng JSON từng dòng) ---
    raw = GAIT_FILE.read_text().strip().splitlines()
    frames = []
    for line in raw:
        line = line.strip()
        if not line:
            continue
        try:
            f = json.loads(line.rstrip(","))
        except Exception:
            continue

        # ép đủ P0..P11
        pose = {}
        for p in PORTS:
            if p in f:
                pose[p] = clamp(f[p])
            else:
                pose[p] = base.get(p, 0)

        # luôn dùng head/tail chuẩn
        for hp, v in FIXED_HEAD_TAIL.items():
            pose[hp] = v

        frames.append(pose)

    if not frames:
        print("No gait frames loaded!")
        return

    print("Loaded", len(frames), "gait frames from file")

    # cắt bỏ đoạn cuối (ngồi xuống)
    if KEEP_FRAMES < len(frames):
        frames = frames[:KEEP_FRAMES]
        print("Trimmed to first", len(frames), "frames (walk only).")

    # đưa robot từ base -> frame đầu tiên
    first = frames[0]
    move_pose(servos, base, first)
    current = first

    # chuẩn bị lắc đầu P10
    head_pitch = HEAD_MIN
    head_dir = +1   # +1: lên tới -70, -1: xuống lại -90

    print("Start forward walk loop… (Ctrl+C to stop)")

    try:
        while True:
            for nxt in frames[1:] + [first]:
                # lắc đầu nhẹ mỗi bước
                head_pitch += head_dir * 1.5
                if head_pitch > HEAD_MAX:
                    head_pitch = HEAD_MAX
                    head_dir = -1
                if head_pitch < HEAD_MIN:
                    head_pitch = HEAD_MIN
                    head_dir = +1

                # ép giá trị P10 cho cả pose_from & pose_to
                current["P10"] = head_pitch
                nxt = dict(nxt)
                nxt["P10"] = head_pitch

                move_pose(servos, current, nxt)
                current = nxt
    except KeyboardInterrupt:
        # về tư thế đứng chuẩn khi thoát
        move_pose(servos, current, base)
        print("\nStopped, back to base pose.")


if __name__ == "__main__":
    main()
