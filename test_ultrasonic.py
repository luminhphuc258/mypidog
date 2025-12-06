import time
from robot_hat import Pin

# CHỌN PIN TRIG và ECHO
TRIG_PIN = 2   # đổi theo chân bạn cắm
ECHO_PIN = 3   # đổi theo chân bạn cắm

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

print("=== TEST ULTRASONIC SENSOR ===")
print(f"Trig = {TRIG_PIN}, Echo = {ECHO_PIN}")
print("Đưa tay lại gần để xem khoảng cách...\n")

def get_distance():
    # Gửi xung TRIGGER 10µs
    trig.off()
    time.sleep_us(2)
    trig.on()
    time.sleep_us(10)
    trig.off()

    # Chờ ECHO lên HIGH
    while echo.value() == 0:
        pulse_start = time.ticks_us()

    # Chờ ECHO xuống LOW
    while echo.value() == 1:
        pulse_end = time.ticks_us()

    duration = time.ticks_diff(pulse_end, pulse_start)

    # Tính khoảng cách (cm)
    distance = (duration * 0.0343) / 2
    return distance


try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.1f} cm")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("DONE.")
