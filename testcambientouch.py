import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

PIN_LEFT  = 27   # OUT1
PIN_RIGHT = 22   # OUT2

GPIO.setup(PIN_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

print("=== TOUCH TEST PUD_DOWN ===")

try:
    while True:
        L = GPIO.input(PIN_LEFT)
        R = GPIO.input(PIN_RIGHT)
        print(f"L={L}  R={R}")
        sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
