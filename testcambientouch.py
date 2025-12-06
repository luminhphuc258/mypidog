import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

PIN_LEFT  = 27   # dây xanh lá
PIN_RIGHT = 22   # dây vàng

GPIO.setup(PIN_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("=== TOUCH SENSOR TEST ===")
print("Chạm vào LEFT hoặc RIGHT để xem giá trị thay đổi")
print("0 = đang chạm, 1 = không chạm\n")

try:
    while True:
        L = GPIO.input(PIN_LEFT)
        R = GPIO.input(PIN_RIGHT)
        print(f"L={L}  R={R}")
        sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
    print("\nThoát.")
