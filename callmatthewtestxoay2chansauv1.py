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

    # góc đích từ file config
    target_p3 = clamp(cfg.get("P3", -20))
    target_p5 = clamp(cfg.get("P5",  20))

    print("Target P3:", target_p3, " | Target P5:", target_p5)

    s3 = Servo("P3")
    s5 = Servo("P5")

    # góc hiện tại (bạn nói): P3 = -20, P5 = 20
    angle3 = -20
    angle5 = 20

    # set lần đầu
    s3.angle(angle3)
    s5.angle(angle5)
    time.sleep(0.3)

    step_delay = 0.03  # chỉnh lớn hơn nếu muốn chậm hơn

    # di chuyển CÙNG LÚC:
    # - P3 chỉ được đi theo chiều âm (giảm dần)
    # - P5 chỉ được đi theo chiều dương (tăng dần)
    while (angle3 > target_p3) or (angle5 < target_p5):
        if angle3 > target_p3:
            angle3 -= 1   # quay theo chiều âm
        if angle5 < target_p5:
            angle5 += 1   # quay theo chiều dương

        s3.angle(clamp(angle3))
        s5.angle(clamp(angle5))

        time.sleep(step_delay)

    print("Done: P3, P5 đã tới góc config.")

if __name__ == "__main__":
    main()
