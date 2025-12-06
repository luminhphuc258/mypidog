#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Danh sách chân còn trống / hay dùng trên Robot HAT
PINS = [2, 3, 4, 5, 6, 12, 13, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]

def scan_echo():
    print("=== SCANNING ULTRASONIC ECHO PIN ===")
    print("Chạm tay gần cảm biến → xem pin nào thay đổi.")

    try:
        while True:
            for pin in PINS:
                try:
                    GPIO.setup(pin, GPIO.IN)
                    val = GPIO.input(pin)
                    if val == 1:   # echo active
                        print(f"[ECHO DETECTED] GPIO {pin} = 1")
                except Exception as e:
                    pass
            time.sleep(0.05)

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Done.")

scan_echo()
