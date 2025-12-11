#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"

# Hai servo chân sau
P5_START = 18     # độ ban đầu
P7_START = -13    # độ ban đầu

DELAY = 0.05      # thời gian chờ giữa mỗi bước tăng 1°
ANGLE_MIN, ANGLE_MAX = -90, 90


def clamp(v):
    return max(ANGLE_MIN, min(ANGLE_MAX, int(v)))


def load_pose_config(path: Path):
    """Load JSON pose config từ file."""
    cfg = {"P5": P5_START, "P7": P7_START}
    if not path.exists():
        print("[WARN] Config file not found, using default.")
        return cfg
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        cfg["P5"] = clamp(data.get("P5", P5_START))
        cfg["P7"] = clamp(data.get("P7", P7_START))
    except Exception as e:
        print(f"[WARN] Cannot parse config: {e}")
    return cfg


def apply_angle(servo, angle):
    """Set servo angle safely with clamp."""
    try:
        servo.angle(clamp(angle))
    except Exception as e:
        print(f"[WARN] servo error: {e}")


def main():

    print("=== STEP 1: Move P5 to +18°, P7 to -13° ===")

    s5 = Servo("P5")
    s7 = Servo("P7")

    apply_angle(s5, P5_START)
    apply_angle(s7, P7_START)

    time.sleep(1.0)

    print("=== STEP 2: Load target from config ===")

    cfg = load_pose_config(POSE_FILE)
    target_P5 = cfg["P5"]
    target_P7 = cfg["P7"]

    print(f"Target P5 → {target_P5}°")
    print(f"Target P7 → {target_P7}°")

    curr_P5 = P5_START
    curr_P7 = P7_START

    print("=== STEP 3: Alternating step movement ===")

    while curr_P5 != target_P5 or curr_P7 != target_P7:

        # ---- Move P5 ----
        if curr_P5 != target_P5:
            if target_P5 > curr_P5:
                curr_P5 += 1
            elif target_P5 < curr_P5:
                curr_P5 -= 1
            apply_angle(s5, curr_P5)
            print(f"P5 → {curr_P5}")
            time.sleep(DELAY)

        # ---- Move P7 ----
        if curr_P7 != target_P7:
            if target_P7 > curr_P7:
                curr_P7 += 1
            elif target_P7 < curr_P7:
                curr_P7 -= 1
            apply_angle(s7, curr_P7)
            print(f"P7 → {curr_P7}")
            time.sleep(DELAY)

    print("=== DONE! P5 & P7 reached config angles ===")


if __name__ == "__main__":
    main()
