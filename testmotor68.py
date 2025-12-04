#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
from robot_hat import Servo

# motor 6 → P5
# motor 8 → P7
MOTOR_PORTS = [
    (6, "P5"),
    (8, "P7"),
]

WIGGLE = 20    # lắc ±20°
HOLD   = 0.4   # thời gian giữ mỗi vị trí (giây)


def clamp(x, lo=-90, hi=90):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def servo_set_angle(servo_obj, angle: int):
    """Thử nhiều API: angle / write / set_angle / setAngle / callable."""
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
    print("=== TEST MOTOR 6 & 8 (P5, P7) ===")
    print("Nhớ chạy bằng: sudo python3 test_motor_6_8.py\n")

    # khởi tạo servo
    servos = {}
    for motor_id, port in MOTOR_PORTS:
        print(f"Khởi tạo motor {motor_id} trên port {port}...")
        servos[port] = Servo(port)
        sleep(0.1)

    # đưa cả 2 về 0° trước
    print("\nĐưa motor 6 (P5) và 8 (P7) về 0°...")
    for motor_id, port in MOTOR_PORTS:
        servo_set_angle(servos[port], 0)
    sleep(1.0)

    # test từng motor
    for motor_id, port in MOTOR_PORTS:
        print(f"\n=== TEST MOTOR {motor_id} trên PORT {port} ===")
        base = 0
        up   = clamp(base + WIGGLE)
        down = clamp(base - WIGGLE)

        print(f"{port}: base {base}°")
        servo_set_angle(servos[port], base)
        sleep(HOLD)

        print(f"{port}: -> {up}°")
        servo_set_angle(servos[port], up)
        sleep(HOLD)

        print(f"{port}: -> {down}°")
        servo_set_angle(servos[port], down)
        sleep(HOLD)

        print(f"{port}: -> back {base}°")
        servo_set_angle(servos[port], base)
        sleep(HOLD + 0.3)

    print("\nHoàn thành test motor 6 & 8.")


if __name__ == "__main__":
    main()
