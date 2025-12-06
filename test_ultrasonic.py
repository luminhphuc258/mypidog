from pidog import Pidog
import time

dog = Pidog()

print("=== TEST Pidog Touch SENSOR ===")
print("Nhấn vào đầu robot để test...\n")

try:
    while True:
        val = dog.touch.read()   # trả về: 0 = không chạm, 1 = trái, 2 = phải, 3 = cả hai
        if val != 0:
            if val == 1:
                print("LEFT touched!")
            elif val == 2:
                print("RIGHT touched!")
            elif val == 3:
                print("BOTH touched!")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("STOP.")
