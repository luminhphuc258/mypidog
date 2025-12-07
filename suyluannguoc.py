#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
stand_sit_stand_robot_hat.py
- KHÔNG dùng pidog
- Chỉ dùng robot_hat.Servo
- Flow: STAND (pose hình) -> SIT mượt -> STAND mượt
- Giữ chân trụ: P0..P3 (không di chuyển khi sit/stand)
- Chân sau: P4,P5 trước, delay 1s, rồi P6,P7
"""

import time
import subprocess
from time import sleep
from robot_hat import Servo


# ===== POSE ĐỨNG (đúng theo hình bạn gửi) =====
STAND_POSE = {
    "P0":  -3,
    "P1":  89,
    "P2":   9,
    "P3": -80,
    "P4":   3,
    "P5":  90,
    "P6":  10,
    "P7": -90,
    "P8": -29,   # head yaw (theo file editor bạn)
    "P9":  90,   # head roll
    "P10": -90,  # head pitch
    "P11":  0,   # tail
}

# ===== Chân trụ (không move khi sit/stand) =====
SUPPORT_LEGS = ["P0", "P1", "P2", "P3"]

# ===== Tốc độ mượt =====
STEP_DELAY = 0.03      # delay giữa mỗi bước nội suy (mượt hơn -> tăng)
DEFAULT_STEPS = 28     # số bước nội suy (mượt hơn -> tăng)
SETTLE_SEC = 1.0       # thời gian đứng yên sau mỗi phase


def clamp(a: float) -> float:
    return max(-90.0, min(90.0, float(a)))


def cleanup_gpio_busy():
    """
    Nếu bạn hay bị 'GPIO busy' ở pidog/gpiozero, free trước cho sạch.
    (robot_hat servo chủ yếu I2C, nhưng cứ free cho an toàn)
    """
    subprocess.run(
        ["bash", "-lc", "sudo fuser -k /dev/gpiochip* /dev/gpiomem /dev/mem 2>/dev/null || true"],
        check=False
    )
    sleep(0.2)


def make_servos(ports):
    s = {}
    for p in ports:
        try:
            s[p] = Servo(p)
        except Exception as e:
            print(f"[WARN] Servo({p}) init fail: {e}")
    return s


def lerp(a, b, t):
    return a + (b - a) * t


def move_ports_smooth(servos, current, target, ports, steps=DEFAULT_STEPS):
    """
    Nội suy mượt các port trong 'ports' từ current -> target.
    current/target là dict {"P4": angle, ...}
    """
    for i in range(1, steps + 1):
        t = i / steps
        for p in ports:
            if p not in servos:
                continue
            a0 = float(current.get(p, 0.0))
            a1 = float(target.get(p, a0))
            ang = clamp(lerp(a0, a1, t))
            servos[p].angle(ang)
        sleep(STEP_DELAY)

    # cập nhật current
    for p in ports:
        if p in target:
            current[p] = float(target[p])


def apply_pose_fast_sync(servos, current, pose, order, steps=18):
    """
    Sync robot về một pose nền (ví dụ STAND) trước khi làm chuyển động.
    """
    move_ports_smooth(servos, current, pose, order, steps=steps)
    sleep(SETTLE_SEC)


def main():
    print("=== STAND -> SIT (smooth) -> STAND (smooth) | robot_hat only ===")
    cleanup_gpio_busy()

    # dùng toàn bộ P0..P11
    all_ports = [f"P{i}" for i in range(12)]
    servos = make_servos(all_ports)

    # current state giả định: unknown -> sync về STAND_POSE để chắc chắn
    current = {p: 0.0 for p in all_ports}

    # 0) Sync về pose đứng trong hình (để “đang đứng” đúng)
    print("[SYNC] Apply STAND pose (from your screenshot)...")
    sync_order = ["P8", "P9", "P10", "P11", "P4", "P5", "P6", "P7", "P0", "P1", "P2", "P3"]
    apply_pose_fast_sync(servos, current, STAND_POSE, sync_order, steps=22)

    # =========================
    # Suy luận “ngược” STAND -> SIT:
    # Giữ P0..P3 làm trụ, chỉ hạ 2 chân sau theo 3 tầng.
    # (Bạn có thể chỉnh các số ở đây nếu muốn ngồi sâu hơn/ít hơn)
    # =========================
    sit_stage_1 = {  # hạ nhẹ
        "P4": 12, "P5": 72,   # rear-left
        "P6": 12, "P7": -72,  # rear-right
    }
    sit_stage_2 = {  # hạ vừa
        "P4": 22, "P5": 52,
        "P6": 22, "P7": -52,
    }
    sit_stage_3 = {  # hạ sâu (ngồi)
        "P4": 32, "P5": 32,
        "P6": 32, "P7": -32,
    }

    # 1) STAND -> SIT (mượt)
    print("[PHASE] STAND -> SIT (keep P0..P3 fixed)")
    # head/tail giữ nguyên (không bắt buộc, nhưng giữ ổn định)
    sleep(0.3)

    # stage 1: P4,P5 trước
    print("  -> Stage1: move P4,P5")
    move_ports_smooth(servos, current, sit_stage_1, ["P4", "P5"], steps=DEFAULT_STEPS)
    print(f"  settle {SETTLE_SEC:.1f}s")
    sleep(SETTLE_SEC)

    # stage 1: P6,P7 sau
    print("  -> Stage1: move P6,P7")
    move_ports_smooth(servos, current, sit_stage_1, ["P6", "P7"], steps=DEFAULT_STEPS)
    print(f"  settle {SETTLE_SEC:.1f}s")
    sleep(SETTLE_SEC)

    # stage 2: P4,P5
    print("  -> Stage2: move P4,P5")
    move_ports_smooth(servos, current, sit_stage_2, ["P4", "P5"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)

    # stage 2: P6,P7
    print("  -> Stage2: move P6,P7")
    move_ports_smooth(servos, current, sit_stage_2, ["P6", "P7"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)

    # stage 3: P4,P5
    print("  -> Stage3: move P4,P5")
    move_ports_smooth(servos, current, sit_stage_3, ["P4", "P5"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)

    # stage 3: P6,P7
    print("  -> Stage3: move P6,P7")
    move_ports_smooth(servos, current, sit_stage_3, ["P6", "P7"], steps=DEFAULT_STEPS)
    print("[OK] Now in SIT-like pose (rear lowered).")
    sleep(SETTLE_SEC)

    # 2) SIT -> STAND (đi ngược lại y chang, mượt)
    print("[PHASE] SIT -> STAND (reverse stages, keep P0..P3 fixed)")

    # reverse stage 2
    print("  -> Back to Stage2: move P4,P5")
    move_ports_smooth(servos, current, sit_stage_2, ["P4", "P5"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)
    print("  -> Back to Stage2: move P6,P7")
    move_ports_smooth(servos, current, sit_stage_2, ["P6", "P7"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)

    # reverse stage 1
    print("  -> Back to Stage1: move P4,P5")
    move_ports_smooth(servos, current, sit_stage_1, ["P4", "P5"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)
    print("  -> Back to Stage1: move P6,P7")
    move_ports_smooth(servos, current, sit_stage_1, ["P6", "P7"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)

    # back to full STAND pose (rear only, vì front trụ giữ nguyên)
    print("  -> Back to STAND rear: move P4,P5")
    move_ports_smooth(servos, current, STAND_POSE, ["P4", "P5"], steps=DEFAULT_STEPS)
    sleep(SETTLE_SEC)
    print("  -> Back to STAND rear: move P6,P7")
    move_ports_smooth(servos, current, STAND_POSE, ["P6", "P7"], steps=DEFAULT_STEPS)

    print("[DONE] Finished: STAND -> SIT -> STAND (robot_hat only).")


if __name__ == "__main__":
    main()
