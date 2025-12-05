#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# Theo log lỗi trước đó: touch_L = GPIO27 (pin 13)
PIN_L = 2      # BCM 27
PIN_R = 3      # tạm đoán touch_R = 22, nếu không đổi thì mình sẽ thử chân khác

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_L, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_R, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("=== RAW GPIO TOUCH TEST ===")
print("Chạm lần lượt vào 2 cảm biến trên đầu.")
print("Nhấn Ctrl+C để dừng.\n")

try:
    while True:
        vL = GPIO.input(PIN_L)
        vR = GPIO.input(PIN_R)
        # v = 1 nghĩa là không chạm (do PULL_UP), 0 là đang chạm (đa số cảm biến active-LOW)
        print(f"L={vL}   R={vR}", end="\r", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n[EXIT] Stop raw GPIO test.")
finally:
    GPIO.cleanup()
