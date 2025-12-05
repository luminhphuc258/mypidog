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
FRAME_DELAY = 0.006      # muốn nhanh hơn có thể giảm 0.004–0.005

# Bỏ bớt một số frame cuối (đoạn ngồi xuống)
TRIM_TAIL_FRAMES = 350   # bỏ 350 frame cuối

# Góc CHUẨN cho head yaw & tail (P8, P9, P11)
HEAD_TAIL_STATIC = {
    "P8": 32,
    "P9": -66,
    "P11": 0,
}

# Lắc đầu trên P10 (head pitch)
HEAD_PITCH_MIN = -90
HEAD_PITCH_MAX = -70
HEAD_PITCH_STEP = 1      # mỗi bước đổi 1 độ

# Lắc “thỉnh thoảng” – ít lắc
HEAD_SHAKE_INTERVAL = 220   # sau ~220 frame mới bắt đầu 1 chu kỳ lắc
HEAD_SHAKE_WINDOW  = 25     # mỗi lần chỉ lắc trong 25 frame


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict, head_pitch: int):
    """
    Gửi góc cho tất cả servo.
    - P0..P7 lấy từ pose (gait frame)
    - P8, P9, P11 dùng góc chuẩn
    - P10 dùng head_pitch (có thể đang lắc hoặc giữ nguyên)
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
        # tạm thêm cho đủ key (sẽ bị override)
        pose.update({
            "P8": 0,
            "P9": 0,
            "P10": 0,
            "P11": 0
        })
        frames.append(pose)

    total = len(frames)
    print(f"Raw gait frames:", total)

    # 1) Bỏ 350 frame cuối
    if TRIM_TAIL_FRAMES > 0 and TRIM_TAIL_FRAMES < total:
        frames = frames[:-TRIM_TAIL_FRAMES]
        print(f"After tail trim: {len(frames)} frames")

    # 2) Tìm frame cuối giống frame đầu nhất để loop mượt
    if len(frames) > 10:
        first = frames[0]
        best_idx = len(frames) - 1
        best_score = float("inf")

        # so sánh từ khoảng giữa về cuối để tìm chu kỳ
        start_search = max(5, len(frames) // 2)
        for i in range(start_search, len(frames)):
            # đo độ lệch P0..P7
            score = 0
            for j in range(8):
                p = f"P{j}"
                diff = frames[i][p] - first[p]
                score += diff * diff
            if score < best_score:
                best_score = score
                best_idx = i

        frames = frames[:best_idx + 1]
        print(f"Cut loop at frame {best_idx}, continuity score = {best_score:.1f}")
        print(f"Final gait length: {len(frames)} frames")

    return frames


def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) Đưa robot về pose chuẩn từ file 1 lần lúc khởi động
    base = load_base_pose()

    # head pitch ban đầu để lắc
    head_pitch = HEAD_PITCH_MIN
    head_dir = +1  # +1 đang ngẩng lên, -1 cúi xuống

    apply_pose(servos, base, head_pitch)
    sleep(0.5)

    # 2) Load toàn bộ frame dáng đi thẳng
    gait_frames = load_gait_frames()
    if not gait_frames:
        print("No gait frames found!")
        return

    # Đưa thẳng robot từ base -> frame đầu tiên
    first = gait_frames[0]
    apply_pose(servos, first, head_pitch)
    sleep(FRAME_DELAY)
    current_index = 0
    frame_counter = 0

    print("Start continuous forward gait with occasional head nod… (Ctrl+C để dừng)")

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
                # phần lớn thời gian giữ ở -90 (không lắc)
                head_pitch = HEAD_PITCH_MIN

            # chuyển frame tiếp theo (loop trơn vì đã cắt đúng điểm)
            current_index += 1
            if current_index >= len(gait_frames):
                current_index = 0  # nhảy về frame 0 nhưng tư thế gần giống → không giật

            pose = gait_frames[current_index]
            apply_pose(servos, pose, head_pitch)
            sleep(FRAME_DELAY)

    except KeyboardInterrupt:
        print("\n Stop by user – trả robot về pose chuẩn.")
        apply_pose(servos, base, HEAD_PITCH_MIN)
        sleep(0.3)


if __name__ == "__main__":
    main()
