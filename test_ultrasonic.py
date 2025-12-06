from robot_hat import Pin
import time

print("=== SCANNING ALL Robot HAT PINS ===")

# Danh sách pin có thể dùng (tự lấy từ thư viện)
VALID_PINS = Pin._Pin__pin.values()

print("Valid Robot HAT Pins:", list(VALID_PINS))

pins = []

for p in VALID_PINS:
    try:
        pins.append((p, Pin(p, Pin.IN)))
    except:
        pass

print("Watching", len(pins), "pins...")
print("Nhấn Ctrl+C để dừng.\n")

last = {}

try:
    while True:
        for p, obj in pins:
            v = obj.value()
            if p not in last or last[p] != v:
                print(f"Pin {p} → {v}")
                last[p] = v
        time.sleep(0.05)

except KeyboardInterrupt:
    print("STOP.")
