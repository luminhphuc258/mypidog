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


# ====== POSE STANDUP (theo hình bạn gửi) ======
# P0..P7 legs, P8..P10 head, P11 tail
TARGET_POSE = {
    "P0":  32,   # leg1
    "P1":  -1,   # leg2
    "P2": -36,   # leg3
    "P3":  17,   # leg4
    "P4":  16,   # leg5
    "P5":  64,   # leg6
    "P6":   3,   # leg7
    "P7": -66,   # leg8
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

    # 1) init dog bằng class của bạn
    #    (nếu constructor của bạn có param khác thì dòng try/except sẽ vẫn chạy được)
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
        print("[DONE] Robot đã về standup pose theo góc bạn đưa.")

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
