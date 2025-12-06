#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# List toàn bộ GPIO khả dụng (BCM)
GPIO_LIST = [2,3,4,5,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22,23,24,25,26,27]

GPIO.setmode(GPIO.BCM)

print("=== SCANNING ALL GPIO FOR ACTIVE SENSOR SIGNALS ===")
print("Nhấn Ctrl + C để dừng...\n")

# Setup tất cả chân làm input có pull-down để tránh nhiễu
for pin in GPIO_LIST:
    try:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    except Exception as e:
        print(f"[SKIP] GPIO {pin}: {e}")

# Lưu trạng thái trước đó để phát hiện thay đổi
prev_state = {pin: GPIO.input(pin) for pin in GPIO_LIST}

try:
    while True:
        for pin in GPIO_LIST:
            try:
                current = GPIO.input(pin)
                if current != prev_state[pin]:
                    print(f"[CHANGE] GPIO {pin} → {current}")
                    prev_state[pin] = current
            except:
                pass

        time.sleep(0.001)

except KeyboardInterrupt:
    print("Stopping scan...")
finally:
    GPIO.cleanup()
