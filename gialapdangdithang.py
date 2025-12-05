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

# tốc độ: mỗi frame dừng rất ngắn -> đi nhanh, không lag
FRAME_DELAY = 0.006

# Bỏ bớt một số frame cuối (đoạn ngồi xuống)
TRIM_TAIL_FRAMES = 350

# ===== POSE ĐỨNG CHUẨN SAU MỖI LOOP =====
STAND_POSE = {
    "P0": -3,
    "P1": 89,
    "P2": 9,
    "P3": -80,
    "P4": 11,
    "P5": 90,
    "P6": 10,
    "P7": -90,
    "P8": -53,
    "P9": 90,
    "P10": -90,
    "P11": 0,
}

# Góc CHUẨN cho head yaw & tail (P8, P9, P11) – trùng với STAND_POSE
HEAD_TAIL_STATIC = {
    "P8": -53,
    "P9": 90,
    "P11": 0,
}

# Lắc đầu trên P10 (head pitch)
HEAD_PITCH_MIN = -90
HEAD_PITCH_MAX = -70
HEAD_PITCH_STEP = 1

# Lắc “thỉnh thoảng”
HEAD_SHAKE_INTERVAL = 220
HEAD_SHAKE_WINDOW = 25

# Thời gian chuyển từ gait -> đứng thẳng và ngược lại
STAND_TRANSITION_SEC = 1.0
STAND_HOLD_SEC = 0.2


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict, head_pitch: int):
    """
    Gửi góc cho tất cả servo.
    - P0..P7 lấy từ pose (gait frame hoặc pose đứng)
    - P8, P9, P11 dùng góc chuẩn HEAD_TAIL_STATIC
    - P10 dùng head_pitch
    """
    send = dict(pose)

    # ép static cho head yaw & tail
    for k, v in HEAD_TAIL_STATIC.items():
        send[k] = v

    # head pitch
    send["P10"] = head_pitch

    for p in PORTS:
        servos[p].angle(clamp(send.get(p, 0)))


def load_base_pose() -> dict:
    data = json.loads(POSE_FILE.read_text())
    base = {k: clamp(v) for k, v in data.items()}
    print("Base pose from config:", base)
    return base


def load_gait_frames():
    raw = GAIT_FILE.read_text()

    # nếu file không có [ ] bọc ngoài thì vá lại cho json.loads
    if not raw.lstrip().startswith("["):
        raw = "[\n" + raw
    if not raw.rstrip().endswith("]"):
        raw = raw.rstrip() + "\n]"

    frames_raw = json.loads(raw)

    frames = []
    for fr in frames_raw:
        pose = {}
        # chắc chắn có đủ P0..P7, P8..P11 sẽ override khi apply
        for i in range(8):
            p = f"P{i}"
            pose[p] = clamp(fr.get(p, 0))
        pose.update({"P8": 0, "P9": 0, "P10": 0, "P11": 0})
        frames.append(pose)

    total = len(frames)
    print("Raw gait frames:", total)

    # Bỏ tail frames
    if 0 < TRIM_TAIL_FRAMES < total:
        frames = frames[:-TRIM_TAIL_FRAMES]
        print(f"After tail trim: {len(frames)} frames")
    else:
        print(f"Loaded {len(frames)} frames (no trim)")

    return frames


def smooth_legs_transition(servos, pose_from, pose_to, head_pitch, duration_sec):
    """
    Nội suy mượt P0..P7 từ pose_from -> pose_to trong duration_sec.
    P8,P9,P11 giữ chuẩn; P10 = head_pitch cố định.
    """
    steps = max(1, int(duration_sec / FRAME_DELAY))
    for s in range(steps + 1):
        t = s / steps
        interp = {}
        for i in range(8):
            p = f"P{i}"
            a = pose_from[p]
            b = pose_to[p]
            interp[p] = a + (b - a) * t
        # placeholder cho P8..P11
        interp.update({"P8": 0, "P9": 0, "P10": 0, "P11": 0})
        apply_pose(servos, interp, head_pitch)
        sleep(duration_sec / steps)


def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) Đưa robot về pose chuẩn trong config 1 lần để sync servo
    base = load_base_pose()
    apply_pose(servos, base, HEAD_PITCH_MIN)
    sleep(0.5)

    # Stand pose cố định sau mỗi loop
    stand_legs = {f"P{i}": STAND_POSE[f"P{i}"] for i in range(8)}

    # head pitch ban đầu
    head_pitch = HEAD_PITCH_MIN
    head_dir = +1

    # 2) Load toàn bộ frame dáng đi thẳng
    gait_frames = load_gait_frames()
    if not gait_frames:
        print("No gait frames found!")
        return

    # Chuyển từ base -> pose đứng chuẩn
    base_legs = {f"P{i}": base.get(f"P{i}", STAND_POSE[f"P{i}"]) for i in range(8)}
    smooth_legs_transition(servos, base_legs, stand_legs,
                           head_pitch=HEAD_PITCH_MIN,
                           duration_sec=STAND_TRANSITION_SEC)

    # Từ đứng chuẩn -> frame đầu tiên
    first = gait_frames[0]
    first_legs = {f"P{i}": first[f"P{i}"] for i in range(8)}
    smooth_legs_transition(servos, stand_legs, first_legs,
                           head_pitch=HEAD_PITCH_MIN,
                           duration_sec=STAND_TRANSITION_SEC / 2)

    current_index = 0
    frame_counter = 0

    print("Start forward gait with stand-pose between loops… (Ctrl+C để dừng)")

    try:
        while True:
            frame_counter += 1

            # --- LẮC ĐẦU THỈNH THOẢNG ---
            if (frame_counter % HEAD_SHAKE_INTERVAL) < HEAD_SHAKE_WINDOW:
                head_pitch += head_dir * HEAD_PITCH_STEP
                if head_pitch >= HEAD_PITCH_MAX:
                    head_pitch = HEAD_PITCH_MAX
                    head_dir = -1
                elif head_pitch <= HEAD_PITCH_MIN:
                    head_pitch = HEAD_PITCH_MIN
                    head_dir = +1
            else:
                head_pitch = HEAD_PITCH_MIN

            pose = gait_frames[current_index]
            apply_pose(servos, pose, head_pitch)
            sleep(FRAME_DELAY)

            if current_index == len(gait_frames) - 1:
                # 1) frame cuối -> pose đứng chuẩn mới
                last_legs = {f"P{i}": pose[f"P{i}"] for i in range(8)}
                smooth_legs_transition(
                    servos,
                    last_legs,
                    stand_legs,
                    head_pitch=HEAD_PITCH_MIN,
                    duration_sec=STAND_TRANSITION_SEC
                )

                # 2) giữ đứng chuẩn
                apply_pose(servos, STAND_POSE, HEAD_PITCH_MIN)
                sleep(STAND_HOLD_SEC)

                # 3) đứng chuẩn -> frame đầu
                smooth_legs_transition(
                    servos,
                    stand_legs,
                    first_legs,
                    head_pitch=HEAD_PITCH_MIN,
                    duration_sec=STAND_TRANSITION_SEC
                )

                current_index = 0
                frame_counter = 0
            else:
                current_index += 1

    except KeyboardInterrupt:
        print("\nStop by user – trả robot về pose đứng chuẩn.")
        apply_pose(servos, STAND_POSE, HEAD_PITCH_MIN)
        sleep(0.3)


if __name__ == "__main__":
    main()
