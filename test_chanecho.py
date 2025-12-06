#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO_LIST = [2,3,4,5,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22,23,24,25,26,27]

print("=== SCANNING GPIO FOR SENSOR SIGNALS ===")
print("Nhấn Ctrl + C để dừng...\n")

prev_state = {}

# Setup từng chân an toàn
for pin in GPIO_LIST:
    try:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        prev_state[pin] = GPIO.input(pin)
    except Exception as e:
        print(f"[SKIP] GPIO {pin}: {e}")

print("\n--- BẮT ĐẦU QUÉT TÍN HIỆU ---\n")

try:
    while True:
        for pin in prev_state.keys():    # chỉ scan pin setup OK
            try:
                current = GPIO.input(pin)
                if current != prev_state[pin]:
                    print(f"[CHANGE] GPIO {pin} → {current}")
                    prev_state[pin] = current
            except:
                pass

        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()
