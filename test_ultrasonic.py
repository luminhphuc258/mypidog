from robot_hat import Pin
import time

print("=== SCAN ALL AVAILABLE Robot HAT PINS ===")

# Thử lấy danh sách PIN từ class Pin
try:
    VALID_PINS = Pin.PIN_LIST
    print("Using Pin.PIN_LIST")
except:
    try:
        VALID_PINS = Pin._pin.values()
        print("Using Pin._pin.values()")
    except:
        raise Exception("Không thể lấy danh sách pin! Thư viện robot_hat quá khác version.")

# Chuyển sang dạng list template
VALID_PINS = list(VALID_PINS)
print("Valid PINs detected:", VALID_PINS)

# Khởi tạo input pin objects
pins = []
for p in VALID_PINS:
    try:
        obj = Pin(p, Pin.IN)
        pins.append((p, obj))
    except Exception as e:
        print(f"Cannot init pin {p}: {e}")

print(f"\nWatching {len(pins)} pins...")
print("Chạm cảm biến → xem pin nào đổi trạng thái\n")

last_state = {}

try:
    while True:
        for p, obj in pins:
            try:
                v = obj.value()
                if last_state.get(p) != v:
                    print(f"Pin {p} = {v}")
                    last_state[p] = v
            except:
                pass
        time.sleep(0.03)

except KeyboardInterrupt:
    print("\nSTOP.")
