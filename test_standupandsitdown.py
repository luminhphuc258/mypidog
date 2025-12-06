#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import subprocess
from time import sleep
from robot_hat import Servo
from pidog import Pidog


# ===== POSE PRE-INIT (theo hình bạn upload) =====
PREPOSE = {
    "P0":  -4,
    "P1":  87,
    "P2":  18,
    "P3": -90,
    "P4":  43,
    "P5": -22,
    "P6": -29,
    "P7":  23,
    "P8":  32,
    "P9": -68,
    "P10": -90,
    "P11":  0,
}

PREPOSE_ORDER = ["P8", "P9", "P10", "P11", "P4", "P5", "P6", "P7", "P0", "P1", "P2", "P3"]


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def apply_pose_robot_hat(pose: dict, order, step_delay=0.03):
    for port in order:
        if port not in pose:
            continue
        ang = clamp(float(pose[port]))
        Servo(port).angle(ang)
        sleep(step_delay)


def cleanup_gpio_busy():
    # Fix lgpio/gpiozero: GPIO busy
    subprocess.run(
        ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
        check=False
    )
    time.sleep(0.2)


def main():
    print("=== PREPOSE -> delay 1s -> cleanup GPIO -> init Pidog ===")

    # 1) PREPOSE
    print("[1] Pre-pose by robot_hat.Servo ...")
    apply_pose_robot_hat(PREPOSE, PREPOSE_ORDER, step_delay=0.03)

    # 2) stabilize
    print("[2] Stabilize 1.0s ...")
    time.sleep(1.0)

    # 3) release GPIO for pidog.reset_mcu()
    print("[3] Cleanup GPIO busy ...")
    cleanup_gpio_busy()

    # 4) init pidog
    print("[4] Init Pidog() ...")
    dog = Pidog()

    if hasattr(dog, "wait_all_done"):
        dog.wait_all_done()

    print("[DONE] OK")
    try:
        dog.close()
    except:
        pass


if __name__ == "__main__":
    main()
