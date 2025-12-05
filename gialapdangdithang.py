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

# mỗi frame dừng rất ngắn -> đi nhanh, bình thường
FRAME_DELAY = 0.006     # thử 0.006s, muốn nhanh hơn nữa thì giảm còn 0.004–0.005

# Góc CHUẨN cho head & tail (không lấy từ gait frame)
HEAD_TAIL_POSE = {
    "P8": 32,
    "P9": -66,
    "P10": -90,
    "P11": 0,
}


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict):
    """Gửi góc cho tất cả servo, P8..P11 luôn dùng góc chuẩn."""
    # copy để không sửa pose gốc
    send = dict(pose)
    # ép head & tail dùng góc chuẩn
    for k, v in HEAD_TAIL_POSE.items():
        send[k] = v

    for p in PORTS:
        servos[p].angle(clamp(send.get(p, 0)))


def load_base_pose() -> dict:
    data = json.loads(POSE_FILE.read_text())
    base = {k: clamp(v) for k, v in data.items()}
    print("Base pose from config:", base)
    return base


def load_gait_frames():
    raw = GAIT_FILE.read_text()

    # nếu file không có [ ] bọc ngoài thì vá lại cho json
    if not raw.lstrip().startswith("["):
        raw = "[\n" + raw
    if not raw.rstrip().endswith("]"):
        raw = raw.rstrip() + "\n]"

    frames_raw = json.loads(raw)

    frames = []
    for fr in frames_raw:
        pose = {}
        # chỉ lấy P0..P7 từ frame; P8..P11 sẽ bị HEAD_TAIL_POSE ghi đè khi apply
        for i in range(8):
            p = f"P{i}"
            pose[p] = clamp(fr.get(p, 0))
        # thêm luôn P8..P11 để pose đầy đủ (nhưng lúc gửi vẫn dùng HEAD_TAIL_POSE)
        pose.update(HEAD_TAIL_POSE)
        frames.append(pose)

    print(f"Loaded {len(frames)} gait frames")
    return frames


def main():
    servos = {p: Servo(p) for p in PORTS}

    # 1) Đưa robot về pose chuẩn từ config 1 LẦN lúc khởi động
    base = load_base_pose()
    apply_pose(servos, base)
    sleep(0.5)

    # 2) Load toàn bộ frame dáng đi thẳng
    gait_frames = load_gait_frames()
    if not gait_frames:
        print("No gait frames found!")
        return

    # Đưa thẳng robot từ base -> frame đầu tiên
    first = gait_frames[0]
    apply_pose(servos, first)
    sleep(FRAME_DELAY)
    current_index = 0

    print("Start continuous forward gait… (Ctrl+C để dừng)")

    try:
        while True:
            # chạy lần lượt 0..N-1, sau đó quay lại 0
            current_index = (current_index + 1) % len(gait_frames)
            pose = gait_frames[current_index]
            apply_pose(servos, pose)
            sleep(FRAME_DELAY)
    except KeyboardInterrupt:
        print("\nStop by user – trả robot về pose chuẩn.")
        apply_pose(servos, base)
        sleep(0.3)


if __name__ == "__main__":
    main()
