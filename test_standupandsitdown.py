#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
sitdown_to_standup_manual_pose.py

Flow:
1) dùng class MatthewPidogBootClass để init & tạo dog object
2) cho dog SIT DOWN bằng pidog.do_action('sit')
3) STAND UP thủ công (KHÔNG dùng pidog) bằng robot_hat.Servo theo pose bạn đưa
"""

import time
from time import sleep

from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass


# ====== POSE STANDUP (POSE MỚI theo ảnh bạn gửi) ======
TARGET_POSE = {
    "P0":  -4,   # leg 1
    "P1":  87,   # leg 2
    "P2":  18,   # leg 3
    "P3": -90,   # leg 4
    "P4":  43,   # leg 5
    "P5": -22,   # leg 6
    "P6": -29,   # leg 7
    "P7":  23,   # leg 8
    "P8":  32,   # head yaw
    "P9": -68,   # head roll
    "P10": -90,  # head pitch
    "P11":  0,   # tail
}

# thứ tự dựng lên để đỡ té: head/tail -> rear legs -> front legs
APPLY_ORDER = ["P8", "P9", "P10", "P11", "P4", "P5", "P6", "P7", "P0", "P1", "P2", "P3"]


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def apply_pose_robot_hat(pose: dict, order=None, step_delay=0.03, settle_sec=1.0):
    """
    Set servo bằng robot_hat.Servo, KHÔNG dùng pidog.
    """
    if order is None:
        order = list(pose.keys())

    servos = {}
    for port in order:
        try:
            servos[port] = Servo(port)
        except Exception as e:
            print(f"[WARN] Không tạo được Servo({port}): {e}")

    for port in order:
        if port not in pose or port not in servos:
            continue
        ang = clamp(float(pose[port]))
        try:
            servos[port].angle(ang)
            sleep(step_delay)
        except Exception as e:
            print(f"[WARN] set {port} -> {ang} fail: {e}")

    if settle_sec and settle_sec > 0:
        print(f"[STABLE] settle {settle_sec:.1f}s...")
        time.sleep(settle_sec)


def main():
    print("=== Sitdown -> Standup (manual pose) ===")

    try:
        boot = MatthewPidogBootClass(cleanup_gpio=True, kill_python=False, enable_prepose=False)
    except TypeError:
        boot = MatthewPidogBootClass()

    dog = boot.create()

    try:
        # 2) SIT DOWN (pidog)
        print("[STEP] Sit down...")
        dog.rgb_strip.set_mode('breath', 'white', bps=0.6)
        dog.do_action("sit", speed=90)
        dog.wait_all_done()
        time.sleep(0.5)

        # stop mọi chuyển động trước khi mình override servo
        if hasattr(dog, "body_stop"):
            dog.body_stop()
        time.sleep(0.2)

        # 3) STAND UP thủ công (robot_hat only)
        print("[STEP] Stand up (manual pose by robot_hat)...")
        dog.rgb_strip.set_mode('boom', 'yellow', bps=2)
        apply_pose_robot_hat(TARGET_POSE, order=APPLY_ORDER, step_delay=0.03, settle_sec=1.0)

        dog.rgb_strip.set_mode('breath', 'white', bps=0.6)
        print("[DONE] Robot đã về standup pose theo góc mới.")

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
