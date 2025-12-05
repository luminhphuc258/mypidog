#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from time import sleep
from pathlib import Path
from robot_hat import Servo

GAIT_FILE = Path.cwd() / "dangdithang_thuvien.txt"

# ==== Speed & Smoothness ====
STEP_DELAY = 0.005     # nhanh gấp đôi
INTERP = 4             # mỗi frame chia nhỏ 4 bước → siêu mượt

# ==== P8–P11: dùng góc chuẩn bạn đưa ====
FIXED_UPPER = {
    "P8": 32,
    "P9": -66,
    "P10_min": -90,
    "P10_max": -70,
    "P11": 0,
}

PORTS = [f"P{i}" for i in range(12)]
for k in ["P8", "P9", "P10", "P11"]:
    if k not in PORTS:
        PORTS.append(k)


def clamp(v):
    return max(-90, min(90, int(v)))


def load_gait_frames():
    raw = GAIT_FILE.read_text().strip()

    # file có thể chứa dấu "," cuối → làm sạch
    if raw.endswith(","):
        raw = raw[:-1]

    frames = json.loads(raw)

    # ==== BỎ FRAME CUỐI: tự động loại frame có dáng ngồi ====
    cleaned = []
    for fr in frames:
        # loại frame bị gập chân: dấu hiệu ngồi
        if fr["P0"] < 10 or fr["P3"] < -20:
            continue
        cleaned.append(fr)

    print(f"Loaded {len(cleaned)} gait frames after cleaning")
    return cleaned


def blend(servos, prev, nxt):
    """Nội suy mượt"""
    for step in range(1, INTERP + 1):
        t = step / INTERP
        pose = {}
        for p in PORTS:
            if p in ["P8", "P9", "P11"]:
                pose[p] = FIXED_UPPER[p]
            elif p == "P10":   # lắc đầu
                pose[p] = clamp(
                    FIXED_UPPER["P10_min"] +
                    (FIXED_UPPER["P10_max"] - FIXED_UPPER["P10_min"]) * t
                )
            else:
                pose[p] = clamp(prev[p] + (nxt[p] - prev[p]) * t)

        for p in PORTS:
            servos[p].angle(pose[p])

        sleep(STEP_DELAY)


def main():
    # chuẩn hóa servo
    servos = {p: Servo(p) for p in PORTS}

    # load frames
    frames = load_gait_frames()
    if not frames:
        print("No frames to run!")
        return

    # === dùng frame đầu tiên làm tư thế chuẩn ===
    base = frames[0].copy()
    base.update({
        "P8": FIXED_UPPER["P8"],
        "P9": FIXED_UPPER["P9"],
        "P10": FIXED_UPPER["P10_min"],
        "P11": FIXED_UPPER["P11"],
    })

    # đưa robot vào dáng chuẩn 1 lần duy nhất
    for p in PORTS:
        servos[p].angle(base[p])
    sleep(0.3)

    print("🚀 Robot walking with cleaned gait + smooth speed + head wobble")

    prev = base
    head_toggle = False

    while True:
        for fr in frames:
            # áp góc cố định P8–P11
            fr2 = fr.copy()
            fr2["P8"] = FIXED_UPPER["P8"]
            fr2["P9"] = FIXED_UPPER["P9"]
            fr2["P11"] = FIXED_UPPER["P11"]

            # lắc đầu
            fr2["P10"] = FIXED_UPPER["P10_max"] if head_toggle else FIXED_UPPER["P10_min"]
            head_toggle = not head_toggle

            blend(servos, prev, fr2)
            prev = fr2


if __name__ == "__main__":
    main()
