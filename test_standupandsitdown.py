#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from time import sleep
from robot_hat import Servo

# =================== POSE A: PRE-POSE (UPDATED THEO HÌNH) ===================
PREPOSE = {
    "P0":  40,  "P1":  87, "P2": -27, "P3": -89,
    "P4":  48,  "P5":  -9, "P6": -29, "P7":  23,
    "P8":  32,  "P9": -68, "P10": -90, "P11": 0,
}

# =================== POSE B: THEO HÌNH BẠN GỬI ===================
POSE_FROM_IMAGE = {
    "P0":  -3,  "P1":  89, "P2":   9, "P3": -80,
    "P4":   3,  "P5":  90, "P6":  10, "P7": -90,
    "P8": -29,  "P9":  90, "P10": -90, "P11": 0,
}

LOCK_PORTS = {"P0", "P1", "P2", "P3"}
PHASE1_MOVE = {"P8", "P9", "P10", "P11", "P4", "P5"}  # P4,P5 trước (+ head/tail)
PHASE2_MOVE = {"P6", "P7"}                            # rồi mới P6,P7

MOVE_STEPS  = 28
FRAME_DELAY = 0.035
SETTLE_SEC  = 1.0


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def lerp(a, b, t):
    return a + (b - a) * t


class SmoothPoseRunner:
    def __init__(self, ports_all):
        self.ports_all = list(ports_all)
        self.servos = {p: Servo(p) for p in self.ports_all}
        self.current = {p: 0.0 for p in self.ports_all}

    def go_to(self, target: dict, move_ports=None, steps=MOVE_STEPS, frame_delay=FRAME_DELAY, settle_sec=SETTLE_SEC):
        if move_ports is None:
            move_ports = set(target.keys())
        else:
            move_ports = set(move_ports)

        for i in range(1, steps + 1):
            t = i / steps
            for port in self.ports_all:
                if port not in move_ports:
                    continue
                if port not in target:
                    continue
                a0 = self.current.get(port, 0.0)
                a1 = float(target[port])
                ang = clamp(lerp(a0, a1, t))
                self.servos[port].angle(ang)
            sleep(frame_delay)

        for port in move_ports:
            if port in target:
                self.current[port] = float(target[port])

        if settle_sec and settle_sec > 0:
            print(f"[STABLE] settle {settle_sec:.1f}s ...")
            time.sleep(settle_sec)


def main():
    print("=== robot_hat ONLY: PREPOSE -> delay -> A->B (P4,P5 first) -> delay -> (P6,P7) ===")

    ports_all = [f"P{i}" for i in range(12)]
    runner = SmoothPoseRunner(ports_all)

    print("[1] PREPOSE (move ALL)...")
    runner.go_to(PREPOSE, move_ports=set(PREPOSE.keys()), settle_sec=1.0)

    print("[2] Delay 1s ...")
    time.sleep(1.0)

    move_phase1 = (PHASE1_MOVE - LOCK_PORTS)
    print("[3a] A->B Phase1: move P4,P5 first (+head/tail), freeze P0-P3 ...")
    runner.go_to(POSE_FROM_IMAGE, move_ports=move_phase1, settle_sec=0.0)

    print("[3b] Delay 1s before moving P6,P7 ...")
    time.sleep(1.0)

    move_phase2 = (PHASE2_MOVE - LOCK_PORTS)
    print("[3c] A->B Phase2: move P6,P7 ...")
    runner.go_to(POSE_FROM_IMAGE, move_ports=move_phase2, settle_sec=1.0)

    print("[DONE] Finished (no pidog).")


if __name__ == "__main__":
    main()
