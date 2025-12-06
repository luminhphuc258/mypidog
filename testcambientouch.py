#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from gpiozero import DigitalInputDevice

# D2 -> GPIO27, D3 -> GPIO22
LEFT_PIN = 27   # dây xanh lá (S1)
RIGHT_PIN = 22  # dây vàng (S2)

# Dual touch của SunFounder thường là "active-low"
# nên mình bật pull_up=True, rồi sẽ tự đảo giá trị sau
left_touch = DigitalInputDevice(LEFT_PIN, pull_up=True)
right_touch = DigitalInputDevice(RIGHT_PIN, pull_up=True)

print("=== RAW DUAL TOUCH TEST ===")
print("Chạm lần lượt vào 2 cảm biến trên đầu.")
print("Nhấn Ctrl+C để dừng.\n")
print("Format:  L=0/1  R=0/1  (1 = đang chạm, 0 = không chạm)\n")

try:
    while True:
        # gpiozero trả .value = 1 khi mức logic HIGH (không chạm, vì pull-up)
        # dual touch thường kéo xuống LOW khi chạm -> mình đảo lại cho dễ hiểu
        L = 1 - int(left_touch.value)
        R = 1 - int(right_touch.value)

        print(f"L={L}  R={R}", end="\r")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[EXIT] Dừng test dual touch.")
