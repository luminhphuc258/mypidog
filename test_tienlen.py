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
        # 1) Stand nhanh
        my_dog.rgb_strip.set_mode('breath', 'white', bps=0.8)
        my_dog.do_action('stand', speed=95)
        my_dog.wait_all_done()
        time.sleep(0.2)

        # 2) Forward nhanh
        my_dog.rgb_strip.set_mode('breath', 'white', bps=0.7)
        my_dog.do_action('forward', step_count=6, speed=99)
        my_dog.wait_all_done()
        time.sleep(0.1)

        # 3) Turn LEFT nhanh (tăng speed + giảm step_count)
        my_dog.rgb_strip.set_mode('boom', 'yellow', bps=3)
        my_dog.do_action('turn_left', step_count=5, speed=99)
        my_dog.wait_all_done()
        time.sleep(0.05)

        # 4) Turn RIGHT nhanh
        my_dog.rgb_strip.set_mode('boom', 'blue', bps=3)
        my_dog.do_action('turn_right', step_count=5, speed=99)
        my_dog.wait_all_done()
        time.sleep(0.05)

        # 5) Sit nhanh
        my_dog.rgb_strip.set_mode('breath', 'red', bps=0.8)
        my_dog.do_action('sit', speed=90)
        my_dog.wait_all_done()

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
