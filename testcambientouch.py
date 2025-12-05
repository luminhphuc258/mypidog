#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from pidog import Pidog
from pidog.dual_touch import TouchStyle

def main():
    print("=== TEST TOUCH SENSOR với Pidog() ===")
    print("Lưu ý: khi chạy script này, servo sẽ bị reset về pose mặc định của Pidog.")
    print("Chạm vào 2 cảm biến trên đầu xem giá trị thay đổi.\n")

    dog = Pidog()     # sẽ reset servo 1 lần
    touch = dog.dual_touch

    try:
        while True:
            val = touch.read()   # số nguyên
            try:
                name = TouchStyle(val).name
            except Exception:
                name = f"UNKNOWN({val})"
            print(f"[TOUCH] raw={val}   name={name}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[EXIT] Dừng test touch.")
    finally:
        try:
            dog.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
