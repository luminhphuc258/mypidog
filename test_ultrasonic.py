#!/usr/bin/env python3
from robot_hat import Pin
import time

# ============================
#  CHỌN CHÂN TRÊN ROBOT HAT
# ============================
TRIG_PIN = 2    # D2  (GPIO02)
ECHO_PIN = 3    # D3  (GPIO03)

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

def get_distance():
    # bảo đảm TRIG = LOW trước
    trig.value(0)
    time.sleep(0.0002)

    # phát xung 10 microsecond
    trig.value(1)
    time.sleep(0.00001)
    trig.value(0)

    # chờ echo lên mức HIGH
    start = time.time()
    while echo.value() == 0:
        start = time.time()

    # chờ echo xuống LOW
    end = time.time()
    while echo.value() == 1:
        end = time.time()

    # tính toán thời gian & khoảng cách
    duration = end - start
    distance_cm = (duration * 34300) / 2
    return distance_cm


print("=== TEST ULTRASONIC on PiDog ===")
print("Nhấn Ctrl+C để dừng.")

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped.")
