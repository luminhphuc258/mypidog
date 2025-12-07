#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from matthewpidogclassinit import MatthewPidogBootClass
from pidog.preset_actions import push_up

def main():
    boot = MatthewPidogBootClass()
    dog = boot.create()  # khởi tạo xong

    # cho servo ổn định chút để đỡ té
    time.sleep(1.0)

    # push up (3 lần)
    push_up(dog, speed=92)
    dog.wait_all_done()
    time.sleep(0.3)

    # stand
    dog.do_action("stand", speed=92)
    dog.wait_all_done()
    time.sleep(0.3)

    # sit
    dog.do_action("sit", speed=92)
    dog.wait_all_done()
     time.sleep(0.3)

    # stand
    dog.do_action("stand", speed=92)
    dog.wait_all_done()
    time.sleep(0.3)

if __name__ == "__main__":
    main()
