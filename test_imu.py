#!/usr/bin/env python3
from pidog import Pidog
from time import sleep

dog = Pidog()
sleep(0.3)

print("=== TEST IMU SH3001 (Accel + Gyro + Temp) ===")
print("Nhấn Ctrl+C để dừng...\n")

while True:
    data = dog.imu.read()   # CHÍNH XÁC API CỦA PIDOG BẢN NÀY
    print(data)
    sleep(0.1)
