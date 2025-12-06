#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from matthewpidogclassinit import MatthewPidogBootClass

def main():
    print("=== Init (MatthewPidogBootClass) -> SIT -> STAND ===")

    boot = MatthewPidogBootClass(
        cleanup_gpio=True,
        kill_python=False,
        enable_prepose=True,
        prepose_delay_sec=1.0
    )

    # Nếu class bạn dùng method khác (vd boot.init()), đổi đúng 1 dòng này:
    dog = boot.create()

    try:
        print("[STABLE] wait 1s ...")
        time.sleep(1.0)

        print("[ACTION] sit ...")
        dog.do_action("sit", speed=90)
        dog.wait_all_done()
        time.sleep(0.6)

        print("[ACTION] stand ...")
        dog.do_action("stand", speed=92)
        dog.wait_all_done()
        time.sleep(0.5)

        print("[DONE]")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        try:
            dog.close()
        except:
            pass

if __name__ == "__main__":
    main()
