#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from robot_hat import Servo
import time

SERVO_PORTS = [f"P{i}" for i in range(12)]

def release_all_servos():
    print("=== RELEASE ALL SERVOS (OFF) ===")
    for p in SERVO_PORTS:
        try:
            s = Servo(p)
            s.disable()        # cách OFF servo đúng nhất (SunFounder Robot HAT)
            print(f"  -> {p} OFF")
        except Exception as e:
            print(f"[WARN] Cannot release {p}: {e}")

    print("All servos released!\n")

if __name__ == "__main__":
    release_all_servos()
