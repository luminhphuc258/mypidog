#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from time import sleep
from pidog import Pidog

# Mapping bạn đã xác nhận
MAP = {
    "FL_HIP": 0,
    "FL_LOWER": 1,
    "FR_HIP": 2,
    "FR_LOWER": 3,
    "RL_HIP": 4,
    "RL_LOWER": 5,
    "RR_HIP": 6,
    "RR_LOWER": 7,
    "NECK_PITCH": 8,
    "NECK_TILT": 9,
    "HEAD_YAW": 10,
    "TAIL": 11,
}

# Pose theo ảnh servo editor (P0..P11)
POSE_BY_CHANNEL = {
    0: -3,
    1:  89,
    2:  9,
    3: -80,
    4:  3,
    5:  90,
    6:  10,
    7: -90,
    8: -90,   # NECK_PITCH  (ảnh: P10=-90 là head pitch, nhưng mapping bạn đưa says pitch=P8)
    9:  90,   # NECK_TILT
    10: -29,  # HEAD_YAW
    11: 0     # TAIL
}

# ---- Nếu pose trong ảnh của bạn là:
# P8(head yaw)=-29, P9(head roll)=+90, P10(head pitch)=-90
# thì tương ứng theo mapping mới:
# HEAD_YAW=P10=-29
# NECK_TILT=P9=+90
# NECK_PITCH=P8=-90
# => Mình đã set ở POSE_BY_CHANNEL đúng theo điều đó.

LEG_PINS  = [MAP["FL_HIP"], MAP["FL_LOWER"], MAP["FR_HIP"], MAP["FR_LOWER"],
             MAP["RL_HIP"], MAP["RL_LOWER"], MAP["RR_HIP"], MAP["RR_LOWER"]]

HEAD_PINS = [MAP["NECK_PITCH"], MAP["NECK_TILT"], MAP["HEAD_YAW"]]
TAIL_PIN  = [MAP["TAIL"]]

leg_init_angles  = [POSE_BY_CHANNEL[p] for p in LEG_PINS]
head_init_angles = [POSE_BY_CHANNEL[p] for p in HEAD_PINS]
tail_init_angle  = POSE_BY_CHANNEL[TAIL_PIN[0]]

print("Init PiDog with mapped pose...")
print("LEG_PINS :", LEG_PINS,  "angles:", leg_init_angles)
print("HEAD_PINS:", HEAD_PINS, "angles:", head_init_angles)
print("TAIL_PIN :", TAIL_PIN,  "angle :", tail_init_angle)

dog = Pidog(
    leg_pins=LEG_PINS,
    head_pins=HEAD_PINS,
    tail_pin=TAIL_PIN,
    leg_init_angles=leg_init_angles,
    head_init_angles=head_init_angles,
    tail_init_angle=tail_init_angle
)

if hasattr(dog, "wait_all_done"):
    dog.wait_all_done()

sleep(1.0)
print("DONE. Nếu mapping đúng, robot sẽ đứng đúng pose bạn lưu.")
