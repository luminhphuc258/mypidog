#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from robot_hat import Servo

# ====== FILE & CẤU HÌNH ======
POSE_FILE = Path.cwd() / "pidog_pose_config.txt"      # file pose đứng hiện tại
GAIT_FILE = Path.cwd() / "dangdithang_thuvien.txt"    # file json bạn record dáng đi thẳng

PORTS      = [f"P{i}" for i in range(12)]   # P0..P11
LEG_PORTS  = [f"P{i}" for i in range(8)]    # chỉ chân P0..P7

CLAMP_LO, CLAMP_HI = -90, 90

# thời gian giữa 2 frame – giảm số này để robot đi nhanh hơn
FRAME_DELAY = 0.015      # 0.02 = chậm, 0.015 = nhanh hơn, 0.01 = rất nhanh
LOOP_COUNT  = 0          # 0 = chạy vô hạn, >0 = số vòng lặp gait

# ====== HÀM TIỆN ÍCH ======
def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))

def apply_pose(servos, pose: dict):
    """Gửi góc đến tất cả servo."""
    for p in PORTS:
        servos[p].angle(clamp(pose[p]))

# ====== MAIN ======
def main():
    # khởi tạo servo
    servos = {p: Servo(p) for p in PORTS}

    # 1) Load base pose từ pidog_pose_config.txt
    base = json.loads(POSE_FILE.read_text())
    for k in base:
        base[k] = clamp(base[k])

    print("Base pose from config:", base)
    apply_pose(servos, base)
    time.sleep(0.5)

    # 2) Load các frame dáng đi thẳng từ file JSON
    #    File: [ { "P0":..., "P1":..., ..., "P7":... }, {...}, ... ]
    gait_frames = json.loads(GAIT_FILE.read_text())
    print(f"Loaded {len(gait_frames)} gait frames")

    # 3) Cho robot đứng đúng tư thế của frame đầu tiên
    pose0 = base.copy()
    for k in LEG_PORTS:
        if k in gait_frames[0]:
            pose0[k] = gait_frames[0][k]
    apply_pose(servos, pose0)
    time.sleep(0.3)

    # 4) Lặp lại các frame để đi thẳng
    loop = 0
    print("Start forward gait replay…")

    while True:
        for frame in gait_frames:
            pose = base.copy()
            # chỉ override 8 chân, giữ nguyên đầu/đuôi theo base
            for k in LEG_PORTS:
                if k in frame:
                    pose[k] = frame[k]
            apply_pose(servos, pose)
            time.sleep(FRAME_DELAY)

        loop += 1
        if LOOP_COUNT > 0 and loop >= LOOP_COUNT:
            break

    # kết thúc: về lại dáng đứng base
    apply_pose(servos, base)
    print("Done, back to base pose.")

if __name__ == "__main__":
    main()
