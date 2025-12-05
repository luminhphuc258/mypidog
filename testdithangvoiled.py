#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo
from pidog.rgb_strip import RGBStrip   # 💡 giống y như file LED riêng của bạn

POSE_FILE = Path.cwd() / "pidog_pose_config.txt"
GAIT_FILE = Path.cwd() / "dangdithang_thuvien.txt"

PORTS = [f"P{i}" for i in range(12)]
CLAMP_LO, CLAMP_HI = -90, 90

FRAME_DELAY = 0.003

TRIM_HEAD_FRAMES = 200
TRIM_TAIL_FRAMES = 350

HEAD_TAIL_STATIC = {
    "P8": 32,
    "P9": -66,
    "P11": 0,
}

HEAD_PITCH_MIN = -90
HEAD_PITCH_MAX = -70
HEAD_PITCH_STEP = 1

HEAD_SHAKE_INTERVAL = 220
HEAD_SHAKE_WINDOW  = 25

STAND_POSE = {
    "P0": -3,
    "P1": 89,
    "P2": 9,
    "P3": -80,
    "P4": 11,
    "P5": 90,
    "P6": 10,
    "P7": -90,
    "P8": -53,
    "P9": 90,
    "P10": -90,
    "P11": 0,
}

STAND_TRANSITION_SEC = 0.7
STAND_HOLD_SEC       = 0.15

NUM_LOOPS = 10


def clamp(x, lo=CLAMP_LO, hi=CLAMP_HI):
    try:
        x = int(round(float(x)))
    except Exception:
        x = 0
    return max(lo, min(hi, x))


def apply_pose(servos, pose: dict, head_pitch: int):
    send = dict(pose)
    for k, v in HEAD_TAIL_STATIC.items():
        send[k] = v
    send["P10"] = head_pitch

    for p in PORTS:
        servos[p].angle(clamp(send.get(p, 0)))


def load_base_pose() -> dict:
    data = json.loads(POSE_FILE.read_text())
    base = {k: clamp(v) for k, v in data.items()}
    print("Base pose from config:", base)
    return base


def load_gait_frames():
    raw = GAIT_FILE.read_text()
    if not raw.lstrip().startswith("["):
        raw = "[\n" + raw
    if not raw.rstrip().endswith("]"):
        raw = raw.rstrip() + "\n]"
    frames_raw = json.loads(raw)

    frames = []
    for fr in frames_raw:
        pose = {}
        for i in range(8):
            p = f"P{i}"
            pose[p] = clamp(fr.get(p, 0))
        pose.update({"P8": 0, "P9": 0, "P10": 0, "P11": 0})
        frames.append(pose)

    total = len(frames)
    print("Raw gait frames:", total)

    if TRIM_HEAD_FRAMES > 0 and TRIM_HEAD_FRAMES < total:
        frames = frames[TRIM_HEAD_FRAMES:]
        print(f"After head trim ({TRIM_HEAD_FRAMES}): {len(frames)} frames")
    else:
        print("No head trim applied.")

    total_after_head = len(frames)
    if TRIM_TAIL_FRAMES > 0 and TRIM_TAIL_FRAMES < total_after_head:
        frames = frames[:-TRIM_TAIL_FRAMES]
        print(f"After tail trim ({TRIM_TAIL_FRAMES}): {len(frames)} frames")
    else:
        print("No tail trim applied or tail trim too large.")

    return frames


def smooth_legs_transition(servos, pose_from, pose_to, head_pitch, duration_sec):
    STEPS_MIN = 15
    STEPS_MAX = 40
    steps = int(duration_sec / 0.02)
    steps = max(STEPS_MIN, min(STEPS_MAX, steps))

    for s in range(steps + 1):
        t = s / steps
        interp = {}
        for i in range(8):
            p = f"P{i}"
            a = pose_from[p]
            b = pose_to[p]
            interp[p] = a + (b - a) * t
        interp.update({"P8": 0, "P9": 0, "P10": 0, "P11": 0})
        apply_pose(servos, interp, head_pitch)
        sleep(duration_sec / steps)


def main():
    servos = {p: Servo(p) for p in PORTS}

    # ==== RGB LED STRIP ====
    # giống như script riêng: tạo strip, set_mode, rồi cứ show() trong loop
    strip = RGBStrip()
    print("Turn LED BLUE (breath) while walking...")
    strip.set_mode(style="breath", color="blue", bps=1.2, brightness=0.8)
    strip.show()

    base = load_base_pose()
    base_legs = {f"P{i}": base[f"P{i}"] for i in range(8)}
    stand_legs = {f"P{i}": STAND_POSE[f"P{i}"] for i in range(8)}

    head_pitch = HEAD_PITCH_MIN
    head_dir = +1

    apply_pose(servos, base, head_pitch)
    sleep(0.5)

    gait_frames = load_gait_frames()
    if not gait_frames:
        print("No gait frames found!")
        # tắt LED rồi thoát
        strip.set_mode(style="solid", color=[0, 0, 0], bps=1, brightness=0)
        strip.show()
        strip.close()
        return

    first = gait_frames[0]
    smooth_legs_transition(
        servos,
        base_legs,
        {f"P{i}": first[f"P{i}"] for i in range(8)},
        head_pitch=HEAD_PITCH_MIN,
        duration_sec=STAND_TRANSITION_SEC / 2
    )

    print(f"Start forward gait – {NUM_LOOPS} loops, LED xanh bật liên tục.")

    frame_counter = 0

    try:
        for loop in range(NUM_LOOPS):
            print(f"== Loop {loop+1}/{NUM_LOOPS} ==")

            for idx, pose in enumerate(gait_frames):
                frame_counter += 1

                # lắc đầu (có thể tắt nếu muốn smooth hơn)
                if (frame_counter % HEAD_SHAKE_INTERVAL) < HEAD_SHAKE_WINDOW:
                    head_pitch += head_dir * HEAD_PITCH_STEP
                    if head_pitch >= HEAD_PITCH_MAX:
                        head_pitch = HEAD_PITCH_MAX
                        head_dir = -1
                    elif head_pitch <= HEAD_PITCH_MIN:
                        head_pitch = HEAD_PITCH_MIN
                        head_dir = +1
                else:
                    head_pitch = HEAD_PITCH_MIN

                apply_pose(servos, pose, head_pitch)

                # cập nhật hiệu ứng LED
                strip.show()

                sleep(FRAME_DELAY)

        # 👉 xong tất cả loop: đứng thẳng rồi tắt LED
        last_pose_legs = {f"P{i}": gait_frames[-1][f"P{i}"] for i in range(8)}

        smooth_legs_transition(
            servos,
            last_pose_legs,
            stand_legs,
            head_pitch=HEAD_PITCH_MIN,
            duration_sec=STAND_TRANSITION_SEC
        )

        apply_pose(servos, STAND_POSE, HEAD_PITCH_MIN)
        sleep(STAND_HOLD_SEC)

        # TẮT LED
        print("Turn LED OFF.")
        strip.set_mode(style="solid", color=[0, 0, 0], bps=1, brightness=0)
        strip.show()
        strip.close()

        print(f"Done {NUM_LOOPS} loops – robot đang đứng thẳng. Kết thúc chương trình.")

    except KeyboardInterrupt:
        print("\nStop by user – trả robot về pose chuẩn từ config.")
        apply_pose(servos, base, HEAD_PITCH_MIN)
        sleep(0.3)

        print("Turn LED OFF (Ctrl+C).")
        strip.set_mode(style="solid", color=[0, 0, 0], bps=1, brightness=0)
        strip.show()
        strip.close()


if __name__ == "__main__":
    main()
