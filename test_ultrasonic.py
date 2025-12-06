from robot_hat import Pin
import robot_hat.pin as pinlib
import time

print("=== SCANNING ALL Robot HAT PINS ===")

# Lấy danh sách PIN AVAILABLE trên Robot HAT
VALID_PINS = list(pinlib.pin_dict.values())
print("Valid pins:", VALID_PINS)

# Khởi tạo mọi pin là INPUT (dù có thể vài pin không đọc được)
pins = []
for p in VALID_PINS:
    try:
        obj = Pin(p, Pin.IN)
        pins.append((p, obj))
    except Exception as e:
        print(f"Cannot init pin {p}: {e}")

print(f"Watching {len(pins)} pins...\n")
print("Touch / kích hoạt cảm biến để xem pin nào thay đổi.")
print("Nhấn Ctrl+C để dừng.\n")

last = {}

try:
    while True:
        for p, obj in pins:
            try:
                v = obj.value()
                if last.get(p) != v:
                    print(f"Pin {p} → {v}")
                    last[p] = v
            except:
                pass
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nSTOP.")
