#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from matthewpidogclassinit import MatthewPidogBootClass
from pidog.preset_actions import push_up

def main():
    print("=== Init PiDog bằng MatthewPidogBootClass -> push_up x3 ===")

    # 1) init dog bằng class của bạn
    boot = MatthewPidogBootClass(
        cleanup_gpio=True,      # tránh GPIO busy
        kill_python=False,
        enable_prepose=True,    # nếu class bạn có prepose
        prepose_delay_sec=1.0
    )

    # tùy class bạn đặt tên hàm tạo dog là gì:
    # - nếu bạn dùng boot.create() thì chạy được
    # - nếu bạn dùng boot.boot() thì đổi lại 1 dòng dưới
    dog = boot.create()

    try:
        # ổn định trước khi làm động tác
        print("[STABLE] wait 1s ...")
        time.sleep(1.0)

        # 2) push_up 3 lần
        for i in range(3):
            print(f"[ACTION] push_up {i+1}/3")
            push_up(dog, speed=92)
            if hasattr(dog, "wait_all_done"):
                dog.wait_all_done()
            time.sleep(0.3)

        print("[DONE] push_up x3 complete.")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            dog.close()
        except:
            pass

if __name__ == "__main__":
    main()
