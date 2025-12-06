import RPi.GPIO as GPIO
import time

# CÁC GPIO KHẢ DỤNG TRÊN ROBOT SHIELD
gpio_list = [2, 3, 4, 17, 22, 27]

# CONFIG
GPIO.setmode(GPIO.BCM)

for pin in gpio_list:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("=== GPIO AUTO SCAN ===")
print("Đang theo dõi các chân:", gpio_list)
print("Hãy chạm cảm biến → xem chân nào thay đổi!")

prev_state = {pin: GPIO.input(pin) for pin in gpio_list}

try:
    while True:
        for pin in gpio_list:
            val = GPIO.input(pin)
            if val != prev_state[pin]:
                print(f"[CHANGE] GPIO {pin}: {prev_state[pin]} → {val}")
                prev_state[pin] = val
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[EXIT] Dừng quét GPIO.")
    GPIO.cleanup()
