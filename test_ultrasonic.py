from robot_hat import Pin
import time

# Danh sách pin INPUT dựa theo Robot HAT SunFounder
VALID_PINS = [2, 3, 4, 5, 6, 12, 13, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]

print("=== MANUAL PIN SCAN FOR ROBOT HAT ===")
print("Watching pins:", VALID_PINS)
print("Chạm cảm biến → xem pin nào thay đổi.\n")

pins = []

# Khởi tạo Pin objects
for p in VALID_PINS:
    try:
        obj = Pin(p, Pin.IN)
        pins.append((p, obj))
    except Exception as e:
        print(f"Pin {p} cannot init: {e}")

last = {}

try:
    while True:
        for p, obj in pins:
            try:
                val = obj.value()
                if last.get(p) != val:
                    print(f"Pin {p} = {val}")
                    last[p] = val
            except:
                pass
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nSTOP.")
