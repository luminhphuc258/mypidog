import RPi.GPIO as GPIO
import time

PIN = 19  # chân SIG từ module

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN)

print("=== TEST PiDog HEAD DISTANCE SENSOR (PWM) ===")

try:
    while True:
        # chờ tín hiệu lên
        while GPIO.input(PIN) == 0:
            pass
        start = time.time()

        # chờ tín hiệu xuống
        while GPIO.input(PIN) == 1:
            pass
        end = time.time()

        pulse_width = end - start
        distance_cm = pulse_width * 1000000 / 58.0   # công thức gần giống HC-SR04 PWM

        print(f"Distance: {distance_cm:.1f} cm")
        time.sleep(0.05)

except KeyboardInterrupt:
    GPIO.cleanup()
