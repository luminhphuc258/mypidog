#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test dual touch sensor trên Robot HAT.

- Lấy tín hiệu từ cổng D2 (LEFT) và D3 (RIGHT).
- In giá trị đọc được ra màn hình liên tục.

Mặc định:
    0 = không chạm
    1 = đang chạm

Nếu thực tế bị ngược (chạm mà in 0, không chạm mà in 1)
thì sửa biến INVERT_LOGIC = True ở dưới.
"""

from time import sleep
from robot_hat import Pin

# Nếu chạm mà giá trị in ra bị ngược, chuyển thành True
INVERT_LOGIC = False   # để test trước, nếu cần thì đổi thành True

def read_touch(pin: Pin) -> int:
    """Đọc digital rồi áp dụng đảo logic nếu cần."""
    raw = pin.read_digital()   # 0 hoặc 1
    if INVERT_LOGIC:
        return 0 if raw else 1
    return raw

def main():
    print("=== RAW DUAL TOUCH TEST (Robot HAT) ===")
    print("Đang dùng:")
    print("  - LEFT  : D2")
    print("  - RIGHT : D3")
    print("Format:  L=0/1  R=0/1   (0 = không chạm, 1 = đang chạm)")
    print("Nhấn Ctrl+C để dừng.\n")

    # Tạo đối tượng Pin từ Robot HAT
    touch_L = Pin("D2")
    touch_R = Pin("D3")

    try:
        while True:
            val_L = read_touch(touch_L)
            val_R = read_touch(touch_R)

            # In trên một dòng cho dễ nhìn
            print(f"\rL={val_L}   R={val_R}   ", end="", flush=True)
            sleep(0.1)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng test dual touch.")

if __name__ == "__main__":
    main()
