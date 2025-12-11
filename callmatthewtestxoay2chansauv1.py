#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
ANGLE_MIN, ANGLE_MAX = -90, 90

def clamp(v, lo=ANGLE_MIN, hi=ANGLE_MAX):
    return max(lo, min(hi, int(v)))

def load_pose_config(path: Path) -> dict:
    cfg = {f"P{i}": 0 for i in range(12)}
    if not path.exists():
        print("[WARN] pose file not found, use all 0")
        return cfg
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            for k, v in data.items():
                if k in cfg:
                    cfg[k] = clamp(v)
    except Exception as e:
        print("[WARN] parse pose file error:", e)
    return cfg

def main():
    cfg = load_pose_config(POSE_FILE)

    # Lấy góc mục tiêu cho P5 và P7 từ file config
    target_p5 = clamp(cfg.get("P5", -5))
    target_p7 = clamp(cfg.get("P7", -5))

    # Góc hiện tại bạn nói
    angle_p5 = -5
    angle_p7 = -5

    print("Target P5:", target_p5, "| Target P7:", target_p7)

    s5 = Servo("P5")
    s7 = Servo("P7")

    # set ngay góc ban đầu
    s5.angle(angle_p5)
    s7.angle(angle_p7)
    time.sleep(0.3)

    step_delay = 0.03   # chỉnh lớn nếu muốn chậm hơn

    # Cả 2 servo xoay cùng một chiều âm (giảm dần)
    while (angle_p5 > target_p5) or (angle_p7 > target_p7):

        if angle_p5 > target_p5:
            angle_p5 -= 1  # quay âm

        if angle_p7 > target_p7:
            angle_p7 -= 1  # quay âm

        s5.angle(clamp(angle_p5))
        s7.angle(clamp(angle_p7))

        time.sleep(step_delay)

    print("DONE: P5 & P7 đã xoay đến góc config.")

if __name__ == "__main__":
    main()
