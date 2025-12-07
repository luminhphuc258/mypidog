#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from time import sleep
from pidog import Pidog

# ====== TUNE Ở ĐÂY ======
Z_START = 35        # thấp (gần tư thế ngồi/khụy)
Z_END   = 80        # cao (tư thế đứng)
Z_STEP  = 1         # bước tăng z
DT      = 0.03      # 30ms ~ 33Hz (êm hơn, đỡ giật)
SERVO_SPEED = 78    # giảm speed để đỡ “đạp” mạnh gây té

# Stand coords (giữ nguyên kiểu trong code bạn gửi)
# coord format: 4 chân, mỗi chân [x, y]
stand_coords = [[[-15, 95], [-15, 95], [5, 90], [5, 90]]]

# Mục tiêu giữ cân bằng
target_pose = {'x': 0, 'y': 0, 'z': Z_START}
target_rpy  = {'roll': 0, 'pitch': 0, 'yaw': 0}

def apply_balance_frame(dog: Pidog, coord, pose, rpy, speed=SERVO_SPEED):
    # PID giữ thân thăng bằng về roll/pitch/yaw mục tiêu
    dog.set_rpy(**rpy, pid=True)
    dog.set_pose(**pose)
    dog.set_legs(coord)
    angles = dog.pose2legs_angle()
    dog.legs.servo_move(angles, speed=speed)

def ramp_standup_balanced(dog: Pidog):
    # 1) vào “low stand” trước để không bật lên quá mạnh
    for _ in range(20):  # giữ 0.6s
        apply_balance_frame(dog, stand_coords[0], target_pose, target_rpy)
        sleep(DT)

    # 2) tăng z từ từ (đây là điểm chính giúp không té)
    z = Z_START
    while z < Z_END:
        z += Z_STEP
        target_pose['z'] = z
        apply_balance_frame(dog, stand_coords[0], target_pose, target_rpy)
        sleep(DT)

    # 3) giữ đứng thêm 1s cho ổn định
    t0 = time.time()
    while time.time() - t0 < 1.0:
        apply_balance_frame(dog, stand_coords[0], target_pose, target_rpy)
        sleep(DT)

def main():
    dog = Pidog()
    sleep(0.5)

    try:
        # SIT trước (preset)
        print("[1] SIT ...")
        dog.do_action('sit', speed=90)
        dog.wait_legs_done()
        sleep(0.6)

        # DỪNG các chuyển động trước khi takeover servo
        if hasattr(dog, "body_stop"):
            dog.body_stop()
        sleep(0.2)

        # SIT -> STAND bằng balance
        print("[2] STAND UP (balanced ramp z) ...")
        ramp_standup_balanced(dog)

        print("[DONE] Balanced stand complete.")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            dog.close()
        except:
            pass

if __name__ == "__main__":
    main()
