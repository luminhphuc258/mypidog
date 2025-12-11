#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import random
import threading
from pathlib import Path

from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass

POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
SERVO_PORTS = [f"P{i}" for i in range(12)]  # P0..P11
DELAY_BETWEEN_WRITES = 0.4   # tăng lên cho chậm hơn nếu muốn
SETTLE_SEC = 1.0
ANGLE_MIN, ANGLE_MAX = -90, 90


def clamp(v, lo=ANGLE_MIN, hi=ANGLE_MAX):
    try:
        v = int(v)
    except Exception:
        v = 0
    return max(lo, min(hi, v))


def servo_set_angle(servo_obj, angle: int):
    angle = clamp(angle)
    for method_name in ("angle", "write", "set_angle", "setAngle"):
        m = getattr(servo_obj, method_name, None)
        if callable(m):
            m(angle)
            return
    if callable(servo_obj):
        servo_obj(angle)
        return
    raise RuntimeError("Servo object has no known angle/write method")


def load_pose_config(path: Path) -> dict:
    cfg = {p: 0 for p in SERVO_PORTS}
    if not path.exists():
        print(f"[WARN] Pose file not found: {path} -> use all zeros.")
        return cfg
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            for k, v in data.items():
                if k in cfg:
                    cfg[k] = clamp(v)
    except Exception as e:
        print(f"[WARN] Pose file parse error: {e} -> use all zeros.")
    return cfg


def apply_pose_config(cfg: dict, step_delay=DELAY_BETWEEN_WRITES, settle_sec=SETTLE_SEC):
    """
    Đưa robot về pose chuẩn:
      - Bước 1: tất cả servo KHÁC P5,P7,P1,P3 set thẳng về config.
      - Bước 2: 2 chân sau (P5, P7) đi CHUNG, rất chậm (nội suy từng ~1 độ).
      - Bước 3: sau khi xong P5,P7 -> mới bắt đầu di chuyển 2 chân trước (P1, P3) từng chân, chậm.
    """
    print("[STEP1] Apply baseline pose from config (robot_hat.Servo)...")

    # Khởi tạo tất cả servo
    servos = {}
    for p in SERVO_PORTS:
        try:
            servos[p] = Servo(p)
        except Exception as e:
            print(f"[WARN] Cannot init Servo({p}): {e}")

    rear_legs = {"P5", "P7"}     # 2 chân sau
    front_legs = {"P1", "P3"}    # 2 chân trước

    # --------- Helper: move pair slow (P5,P7) ----------
    def move_pair_slow(port_a, port_b):
        if port_a not in servos or port_b not in servos:
            print(f"[WARN] Missing servo {port_a} or {port_b}, skip slow pair.")
            return

        s_a = servos[port_a]
        s_b = servos[port_b]

        target_a = clamp(cfg.get(port_a, 0))
        target_b = clamp(cfg.get(port_b, 0))

        # Giả định start ~0 độ
        curr_a = 0
        curr_b = 0

        da = target_a - curr_a
        db = target_b - curr_b

        steps = int(max(abs(da), abs(db)))  # ~1 độ/bước
        if steps <= 0:
            servo_set_angle(s_a, target_a)
            servo_set_angle(s_b, target_b)
            time.sleep(step_delay)
            return

        print(f"  -> Slow pair {port_a}/{port_b}: {steps} mini-steps")
        for i in range(steps + 1):
            frac = i / float(steps)
            angle_a = curr_a + da * frac
            angle_b = curr_b + db * frac
            try:
                servo_set_angle(s_a, angle_a)
                servo_set_angle(s_b, angle_b)
            except Exception as e:
                print(f"[WARN] move_pair_slow {port_a}/{port_b}: {e}")
                break
            time.sleep(step_delay)

    # --------- Helper: move single leg slow (P1 or P3) ----------
    def move_single_slow(port):
        if port not in servos:
            print(f"[WARN] Missing servo {port}, skip slow move.")
            return

        s = servos[port]
        target = clamp(cfg.get(port, 0))

        curr = 0   # giả định từ 0; nếu muốn, có thể đổi thành góc bạn đo thực tế
        d = target - curr
        steps = int(abs(d))
        if steps <= 0:
            servo_set_angle(s, target)
            time.sleep(step_delay)
            return

        print(f"  -> Slow leg {port}: {steps} mini-steps")
        for i in range(steps + 1):
            frac = i / float(steps)
            angle = curr + d * frac
            try:
                servo_set_angle(s, angle)
            except Exception as e:
                print(f"[WARN] move_single_slow {port}: {e}")
                break
            time.sleep(step_delay)

    # ===== BƯỚC 1: set tất cả servo KHÁC chân sau & chân trước =====
    for p in SERVO_PORTS:
        if p not in servos:
            continue
        if p in rear_legs or p in front_legs:
            # P5,P7,P1,P3 tạm thời chưa đụng
            continue
        try:
            servo_set_angle(servos[p], cfg.get(p, 0))
            time.sleep(step_delay)
        except Exception as e:
            print(f"[WARN] Apply {p} failed: {e}")

    # ===== BƯỚC 2: Di chuyển RẤT CHẬM 2 chân sau (P5, P7) cùng lúc =====
    print("  -> Slowly move REAR legs together (P5 & P7)...")
    move_pair_slow("P5", "P7")

    # ===== BƯỚC 3: SAU KHI XONG P5,P7 MỚI BẮT ĐẦU DI CHUYỂN P1,P3 =====
    print("  -> Slowly move FRONT legs (P1 then P3)...")
    move_single_slow("P1")
    move_single_slow("P3")

    # ===== Chờ ổn định =====
    if settle_sec and settle_sec > 0:
        print(f"[STABLE] settle {settle_sec:.1f}s ...")
        time.sleep(settle_sec)


# ===================== HEAD LOCK + WIGGLE THREAD =====================

def start_head_controller(
    p8_fixed=90,          # giữ nguyên P8
    p9_fixed=-90,         # giữ nguyên P9
    p10_min=-80,          # P10 lắc từ -80
    p10_max=80,           # tới +80
    write_interval=0.08,  # tốc độ ghi
    hold_range=(0.5, 1.2)
):
    """
    - P8: luôn giữ p8_fixed
    - P9: luôn giữ p9_fixed
    - P10: lắc qua lại giữa p10_min và p10_max
    """
    stop_evt = threading.Event()

    try:
        s8 = Servo("P8")
        s9 = Servo("P9")
        s10 = Servo("P10")
    except Exception as e:
        print(f"[WARN] Cannot init head servos P8/P9/P10: {e}")
        return stop_evt, None

    def worker():
        # set lần đầu
        try:
            s8.angle(clamp(p8_fixed))
            s9.angle(clamp(p9_fixed))
            s10.angle(clamp(p10_min))
        except Exception:
            pass

        target = p10_min
        next_flip = time.time() + random.uniform(*hold_range)

        while not stop_evt.is_set():
            now = time.time()

            if now >= next_flip:
                # đổi hướng lắc
                target = p10_max if target == p10_min else p10_min
                next_flip = now + random.uniform(*hold_range)

            try:
                s8.angle(clamp(p8_fixed))
                s9.angle(clamp(p9_fixed))
                s10.angle(clamp(target))
            except Exception:
                pass

            time.sleep(write_interval)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return stop_evt, t


def main():
    print("=== SIMPLE HEAD + SLOW REAR LEGS TEST ===")

    # STEP 1: trả robot về pose chuẩn trong file (có slow move cho chân sau & sau đó chân trước)
    cfg = load_pose_config(POSE_FILE)
    apply_pose_config(cfg, step_delay=DELAY_BETWEEN_WRITES, settle_sec=1.0)

    # STEP 2: khởi tạo Pidog qua MatthewPidogBootClass
    print("[STEP2] Boot Pidog via MatthewPidogBootClass...")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)

    # STEP 3: cho robot đứng lên chuẩn
    print("[STEP3] dog.stand() ...")
    dog.do_action("stand", speed=30)
    dog.wait_all_done()
    time.sleep(0.5)

    # STEP 4: bắt đầu lắc đầu
    print("[STEP4] Start head wiggle (P8,P9 fixed; P10 -60..+60). Ctrl+C to stop.")
    head_stop_evt, head_thread = start_head_controller(
        p8_fixed=32,
        p9_fixed=-90,
        p10_min=-60,
        p10_max=60,
        write_interval=0.08,
        hold_range=(0.6, 1.5),
    )

    try:
        while True:
            time.sleep(0.5)   # chỉ giữ cho chương trình sống
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C pressed, stopping head thread...")
    finally:
        if head_stop_evt is not None:
            head_stop_evt.set()
        if head_thread is not None:
            head_thread.join(timeout=0.5)
        print("[DONE] Head + slow legs test finished.")


if __name__ == "__main__":
    main()
