#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test dual touch sensor trên Robot HAT.

- LEFT  : D2
- RIGHT : D3

Hiển thị:
  L=0/1  R=0/1   (0 = không chạm, 1 = đang chạm)

Nếu logic bị ngược (chạm = 0, không chạm = 1) thì đổi INVERT_LOGIC = True.
"""

from time import sleep
from robot_hat import Pin

INVERT_LOGIC = False   # nếu thấy chạm mà ra 0 thì đổi thành True


def read_touch(pin: Pin) -> int:
    """
    Đọc digital từ pin với nhiều kiểu khác nhau
    tuỳ version robot_hat trên máy.
    """
    # Ưu tiên các method có sẵn nếu tồn tại
    if hasattr(pin, "read_digital"):
        raw = pin.read_digital()
    elif hasattr(pin, "read"):
        raw = pin.read()
    elif hasattr(pin, "value"):
        raw = pin.value()
    else:
        # hết cách thì thử gọi như hàm
        raw = pin()

    # Chuẩn hoá về 0/1
    raw = 1 if raw else 0

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

    # In thử kiểu pin để debug nếu cần
    print(f"[DEBUG] type(touch_L) = {type(touch_L)}")

    try:
        while True:
            val_L = read_touch(touch_L)
            val_R = read_touch(touch_R)

            print(f"\rL={val_L}   R={val_R}   ", end="", flush=True)
            sleep(0.1)

    except KeyboardInterrupt:
        print("\n[EXIT] Dừng test dual touch.")


if __name__ == "__main__":
    main()
