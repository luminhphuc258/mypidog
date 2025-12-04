#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from pathlib import Path
from robot_hat import Servo

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"

# ========= TUNING (chỉnh ở đây) =========
CLAMP_LO, CLAMP_HI = -90, 90

# Tư thế đứng: giống code bạn đang OK
STAND_DELTA = 70

# Đi tới: biên độ & tốc độ
LEAN_DELTA = 10       # nghiêng người để dồn trọng tâm (nhỏ thôi để đỡ ngã)
STEP_DELTA = 18       # bước (tăng/giảm góc cho cặp chân đang bước)
STEP_TIME  = 0.18     # thời gian cho 1 pha (giảm để nhanh hơn, tăng để chậm hơn)
RAMP_STEPS = 22       # mượt khi chuyển pose
RAMP_DT    = 0.015

# Nếu robot đi lùi -> đổi dấu này
FWD_SIGN = +1         # +1 hoặc -1

# Chỉ dùng 8 servo chân + thêm P10 giữ đầu cố định (không bắt buộc)
LEG_PORTS = [f"P{i}" for i in range(8)]
HEAD_PITCH_PORT = "P10"   # bạn đang dùng P10
USE_HEAD_P10 = True

# Các servo "được phép move" (motor 1,3,5,7) -> ports P0,P2,P4,P6
MOVE_LEG_INDEXES = [0, 2, 4, 6]

# Bạn đã nói motor 3 và 7 đang ngược chiều => P2 và P6 = -1
# (index theo P0..P7)
LEG_DIR = [
    +1,  # P0 move
    +1,  # P1 locked
    -1,  # P2 move (reversed)
    +1,  # P3 locked
    +1,  # P4 move
    +1,  # P5 locked
    -1,  # P6 move (reversed)
    +1,  # P7 locked
]

# Pace gait: đi bên phải rồi bên trái
# NOTE: bạn có thể đảo RIGHT/LEFT nếu thấy thực tế ngược bên.
RIGHT_PAIR = [2, 6]   # P2, P6
LEFT_PAIR  = [0, 4]   # P0, P4


# ========= helpers =========
def clamp(v, lo=CLAMP_LO, hi=CLAMP_HI):
    v = int(round(v))
    return max(lo, min(hi, v))

def lerp(a, b, t: float):
    return a + (b - a) * t

def load_pose(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    # đảm bảo đủ key P0..P11 nếu thiếu thì tự 0
    for i in range(12):
        data.setdefault(f"P{i}", 0)
    # clamp
    for k, v in list(data.items()):
        if k.startswith("P"):
            data[k] = clamp(v)
    return data

def make_stand_pose(base_pose: dict) -> dict:
    stand = dict(base_pose)
    for i in range(8):
        p = f"P{i}"
        if i in MOVE_LEG_INDEXES:
            stand[p] = clamp(base_pose[p] + LEG_DIR[i] * STAND_DELTA)
        else:
            stand[p] = base_pose[p]
    # giữ head pitch y như base (để không “giật” đầu)
    if USE_HEAD_P10:
        stand[HEAD_PITCH_PORT] = base_pose[HEAD_PITCH_PORT]
    return stand

def apply_pose(servos: dict, pose: dict):
    for p, s in servos.items():
        s.angle(clamp(pose.get(p, 0)))

def move_pose(servos: dict, pose_from: dict, pose_to: dict, steps=RAMP_STEPS, dt=RAMP_DT):
    for s in range(1, steps + 1):
        t = s / steps
        for p, servo in servos.items():
            a = pose_from.get(p, 0)
            b = pose_to.get(p, 0)
            servo.angle(clamp(lerp(a, b, t)))
        time.sleep(dt)

def pose_with_lean(stand_pose: dict, lean_to: str) -> dict:
    """
    lean_to: 'left' hoặc 'right'
    Nghiêng bằng cách:
      - bên mình muốn nghiêng: +LEAN
      - bên còn lại: -LEAN
    (tất cả đều nhân với LEG_DIR để đúng chiều từng servo)
    """
    out = dict(stand_pose)
    if lean_to == "left":
        up = set(LEFT_PAIR)
        dn = set(RIGHT_PAIR)
    else:
        up = set(RIGHT_PAIR)
        dn = set(LEFT_PAIR)

    for i in MOVE_LEG_INDEXES:
        p = f"P{i}"
        if i in up:
            out[p] = clamp(stand_pose[p] + LEG_DIR[i] * LEAN_DELTA)
        elif i in dn:
            out[p] = clamp(stand_pose[p] - LEG_DIR[i] * LEAN_DELTA)
    return out

def pose_step_pair(lean_pose: dict, pair: list[int], step_delta: int) -> dict:
    """
    Cho cặp chân 'pair' bước: cộng thêm step_delta (có FWD_SIGN)
    """
    out = dict(lean_pose)
    for i in pair:
        p = f"P{i}"
        out[p] = clamp(lean_pose[p] + LEG_DIR[i] * (FWD_SIGN * step_delta))
    return out

def hold_update(servos: dict, seconds: float, pose: dict):
    """
    Một số board cần giữ update nhẹ, mình chỉ sleep là đủ.
    """
    apply_pose(servos, pose)
    time.sleep(seconds)


def main():
    base = load_pose(POSE_FILE)           # pose chuẩn bạn đã cân
    stand = make_stand_pose(base)

    # chỉ giữ chân + P10 (đầu)
    used_ports = set(LEG_PORTS)
    if USE_HEAD_P10:
        used_ports.add(HEAD_PITCH_PORT)

    servos = {p: Servo(p) for p in sorted(used_ports, key=lambda x: int(x[1:]))}

    print("Loaded BASE (sit/neutral) from:", POSE_FILE)
    print("MOVE_LEG_INDEXES:", MOVE_LEG_INDEXES, "=>", [f"P{i}" for i in MOVE_LEG_INDEXES])
    print("RIGHT_PAIR:", RIGHT_PAIR, "LEFT_PAIR:", LEFT_PAIR, "FWD_SIGN:", FWD_SIGN)

    # 1) về base pose mượt
    move_pose(servos, base, base, steps=10, dt=0.01)
    hold_update(servos, 0.4, base)

    # 2) đứng lên mượt (đúng kiểu bạn đang OK)
    move_pose(servos, base, stand, steps=28, dt=0.015)
    hold_update(servos, 0.5, stand)

    # 3) gait pace 5 giây
    t0 = time.time()
    while time.time() - t0 < 5.0:
        # --- bước bên phải ---
        leanL = pose_with_lean(stand, "left")                 # dồn trọng tâm sang trái
        stepR = pose_step_pair(leanL, RIGHT_PAIR, STEP_DELTA) # cặp phải bước
        backR = leanL                                         # trả về sau bước

        move_pose(servos, stand, leanL, steps=10, dt=0.012)
        hold_update(servos, STEP_TIME * 0.6, leanL)

        move_pose(servos, leanL, stepR, steps=10, dt=0.012)
        hold_update(servos, STEP_TIME, stepR)

        move_pose(servos, stepR, backR, steps=10, dt=0.012)
        hold_update(servos, STEP_TIME * 0.5, backR)

        # --- bước bên trái ---
        leanR = pose_with_lean(stand, "right")                # dồn trọng tâm sang phải
        stepL = pose_step_pair(leanR, LEFT_PAIR, STEP_DELTA)  # cặp trái bước
        backL = leanR

        move_pose(servos, backR, leanR, steps=10, dt=0.012)
        hold_update(servos, STEP_TIME * 0.6, leanR)

        move_pose(servos, leanR, stepL, steps=10, dt=0.012)
        hold_update(servos, STEP_TIME, stepL)

        move_pose(servos, stepL, backL, steps=10, dt=0.012)
        hold_update(servos, STEP_TIME * 0.5, backL)

        # quay về stand để tránh drift
        move_pose(servos, backL, stand, steps=8, dt=0.012)
        hold_update(servos, 0.05, stand)

    # kết thúc: đứng yên
    hold_update(servos, 0.3, stand)

    # close
    for s in servos.values():
        try:
            s.angle(0)  # optional: hoặc bỏ dòng này nếu bạn muốn giữ nguyên
        except Exception:
            pass
    for s in servos.values():
        try:
            s.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass