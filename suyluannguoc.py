#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
stand_sit_stand_robot_hat.py
- KHÔNG dùng pidog
- Chỉ dùng robot_hat.Servo
- Flow: STAND (pose hình) -> SIT mượt -> STAND mượt
- Giữ chân trụ: P0..P3 (không di chuyển khi sit/stand)
- Chân sau: P4,P5 trước, delay 1s, rồi P6,P7
"""

import time
import subprocess
from time import sleep
from robot_hat import Servo


# ===== POSE ĐỨNG (đúng theo hình bạn gửi) =====
STAND_POSE = {
    "P0":  -3,
    "P1":  89,
    "P2":   9,
    "P3": -80,
    "P4":   3,
    "P5":  90,
    "P6":  10,
    "P7": -90,
    "P8": -29,   # head yaw (theo file editor bạn)
    "P9":  90,   # head roll
    "P10": -90,  # head pitch
    "P11":  0,   # tail
}

# ===== Chân trụ (không move khi sit/stand) =====
SUPPORT_LEGS = ["P0", "P1", "P2", "P3"]

# ===== Tốc độ mượt =====
STEP_DELAY = 0.03      # delay giữa mỗi bước nội suy (mượt hơn -> tăng)
DEFAULT_STEPS = 28     # số bước nội suy (mượt hơn -> tăng)
SETTLE_SEC = 1.0       # thời gian đứng yên sau mỗi phase


def clamp(a: float) -> float:
    return max(-90.0, min(90.0, float(a)))


def cleanup_gpio_busy():
    """
    Nếu bạn hay bị 'GPIO busy' ở pidog/gpiozero, free trước cho sạch.
    (robot_hat servo chủ yếu I2C, nhưng cứ free cho an toàn)
    """
    subprocess.run(
        ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
        check=False
    )
    sleep(0.2)


def make_servos(ports):
    s = {}
    for p in ports:
        tr
