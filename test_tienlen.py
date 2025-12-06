#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
from matthewpidogclassinit import MatthewPidogBootClass


def move_forward(dog, duration=2.0, speed=40):
    """
    Thử nhiều API khác nhau để đi tới, phù hợp nhiều version pidog.
    """
    # 1) nếu có method go_forward / forward
    for name in ("go_forward", "forward", "move_forward"):
        if hasattr(dog, name):
            fn = getattr(dog, name)
            try:
                print(f"[MOVE] Using dog.{name}({speed}) for {duration}s")
                fn(speed)
                sleep(duration)
                # stop nếu có
                if hasattr(dog, "stop"):
                    dog.stop()
                return True
            except TypeError:
                # một số hàm không nhận speed
                print(f"[MOVE] Using dog.{name}() for {duration}s")
                fn()
                sleep(duration)
                if hasattr(dog, "stop"):
                    dog.stop()
                return True
            except Exception as e:
                print("[MOVE WARN]", e)

    # 2) nếu hỗ trợ do_action
    if hasattr(dog, "do_action"):
        for action in ("forward", "walk_forward", "step_forward", "trot_forward"):
            try:
                print(f"[MOVE] Using dog.do_action('{action}', speed={speed})")
                dog.do_action(action, speed=speed)
                if hasattr(dog, "wait_all_done"):
                    dog.wait_all_done()
                else:
                    sleep(duration)
                return True
            except Exception as e:
                # action không tồn tại thì thử action khác
                pass

    print("[ERROR] Không tìm thấy API đi tới trong version pidog này.")
    print("Bạn chạy: python3 -c \"from pidog import Pidog; import inspect; print([m for m in dir(Pidog) if 'for' in m or 'walk' in m or 'move' in m])\"")
    return False


def main():
    boot = PidogBootstrap(
        pose_file="pidog_pose_config.txt",
        enable_force_head=True,
        force_head_port="P10",
        force_head_angle=-90,
    )

    dog = boot.create()

    print("\n=== TEST: Move forward ===")
    ok = move_forward(dog, duration=2.0, speed=40)

    if ok:
        print("[DONE] Forward test finished.")
    else:
        print("[FAIL] Forward test failed.")


if __name__ == "__main__":
    main()
