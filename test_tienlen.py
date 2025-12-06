#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from matthewpidogclassinit import MatthewPidogBootClass


def main():
    boot = MatthewPidogBootClass(
        pose_file="pidog_pose_config.txt",
        enable_force_head=True,
        force_head_port="P10",
        force_head_angle=-90,
    )
    my_dog = boot.create()

    try:
        # 1) Stand
        my_dog.rgb_strip.set_mode('breath', 'white', bps=0.6)
        my_dog.do_action('stand', speed=80)
        my_dog.wait_all_done()
        time.sleep(0.4)

        # 2) Forward a bit
        my_dog.rgb_strip.set_mode('breath', 'white', bps=0.5)
        my_dog.do_action('forward', step_count=4, speed=95)
        my_dog.wait_all_done()
        time.sleep(0.2)

        # 3) Turn right one round (thường step_count càng lớn càng xoay nhiều)
        my_dog.rgb_strip.set_mode('boom', 'blue', bps=2)
        my_dog.do_action('turn_right', step_count=12, speed=90)
        my_dog.wait_all_done()
        time.sleep(0.2)

        # 4) Turn left (ít hơn 1 vòng)
        my_dog.rgb_strip.set_mode('boom', 'yellow', bps=2)
        my_dog.do_action('turn_left', step_count=6, speed=90)
        my_dog.wait_all_done()
        time.sleep(0.2)

        # 5) Sit down
        my_dog.rgb_strip.set_mode('breath', 'red', bps=0.6)
        my_dog.do_action('sit', speed=70)
        my_dog.wait_all_done()
        time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        try:
            my_dog.close()
        except:
            pass


if __name__ == "__main__":
    main()
