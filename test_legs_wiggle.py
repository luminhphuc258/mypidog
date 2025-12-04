#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
from robot_hat import Servo

# Mapping chiều chân
LEG_DIR = [
    1,   # P0 (motor 1)
    1,   # P1 (motor 2)
   -1,   # P2 (motor 3)  (reversed)
    1,   # P3 (motor 4)
    1,   # P4 (motor 5)
    1,   # P5 (motor 6)
   -1,   # P6 (motor 7)  (reversed)
    1    # P7 (motor 8)
]

PORTS = [f"P{i}" for i in range(8)]   # test 8 chân
WIGGLE = 20                           # lắc ±20 độ
HOLD = 0.35


def clamp(x, lo=-90, hi=90):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def servo_set_angle(servo_obj, angle: int):
    """Thử nhiều API khác nhau: angle / write / set_angle / setAngle / callable."""
    angle = clamp(angle)
    for name in ("angle", "write", "set_angle", "setAngle"):
        m = getattr(servo_obj, name, None)
        if callable(m):
            m(angle)
            return
    if callable(servo_obj):
        servo_obj(angle)
        return
    raise RuntimeError("Servo object has no known angle/write method")


def main():
    print("=== SIMPLE LEG TEST P0..P7 ===")
    print("Chạy bằng: sudo python3 test_legs_simple.py\n")

    # Khởi tạo servo
    servos = {p: Servo(p) for p in PORTS}

    # Đưa tất cả về 0 trước
    print("Đưa tất cả P0..P7 về 0°")
    for p in PORTS:
        try:
            servo_set_angle(servos[p], 0)
        except Exception as e:
            print(f"[ERR] Không điều khiển được {p}: {e}")
    sleep(1.0)

    # Test từng chân
    for i, p in enumerate(PORTS):
        direction = LEG_DIR[i]
        reversed_flag = " (reversed)" if direction == -1 else ""
        print(f"\n=== TEST MOTOR {i+1} -> PORT {p} | LEG_DIR={direction}{reversed_flag} ===")

        base = 0
        up   = clamp(base + direction * WIGGLE)
        down = clamp(base - direction * WIGGLE)

        # về base
        print(f"{p}: base {base}°")
        servo_set_angle(servos[p], base)
        sleep(HOLD)

        print(f"{p}: -> {up}°")
        servo_set_angle(servos[p], up)
        sleep(HOLD)

        print(f"{p}: -> {down}°")
        servo_set_angle(servos[p], down)
        sleep(HOLD)

        print(f"{p}: -> back {base}°")
        servo_set_angle(servos[p], base)
        sleep(HOLD + 0.2)

    print("\nHoàn thành test P0..P7.")


if __name__ == "__main__":
    main()
