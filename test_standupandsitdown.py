#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
import time
from pathlib import Path
from time import sleep
from robot_hat import Servo


# ===================== FILE CONFIG =====================
POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"


# ===================== POSE A / POSE B (THEO 2 HÌNH BẠN GỬI) =====================
POSE_A = {
    "P0":  0,
    "P1":  18,
    "P2":  4,
    "P3":  0,
    "P4":  8,
    "P5":  70,
    "P6":  3,
    "P7": -64,
    "P8": -32,
    "P9":  90,
    "P10": -90,
    "P11":  0,
}

POSE_B = {
    "P0":  22,
    "P1":  40,
    "P2": -46,
    "P3": -26,
    "P4":  8,
    "P5":  70,
    "P6":  3,
    "P7": -64,
    "P8": -32,
    "P9":  90,
    "P10": -90,
    "P11":  0,
}


# ===================== SPEED TUNING (NHẸ / KHÔNG NGÃ) =====================
MOVE_STEPS = 40          # càng lớn càng mượt (chậm hơn)
FRAME_DELAY = 0.04       # delay giữa mỗi step
SETTLE_SEC = 0.8         # settle sau khi xong 1 pose

INTER_PHASE_DELAY_1 = 1.0  # delay 1s giữa (P4,P5) và (P6,P7) đúng ý bạn


# ===================== SAFE PHASES =====================
PHASES = [
    ["P8", "P9", "P10", "P11"],  # head + tail trước
    ["P4", "P5"],                # rear left
    ["__DELAY1__"],              # delay 1s
    ["P6", "P7"],                # rear right
    ["P0", "P1"],                # front left
    ["P2", "P3"],                # front right
]


def clamp(a: float) -> float:
    if a < -90: return -90
    if a > 90:  return 90
    return a


def lerp(a, b, t):
    return a + (b - a) * t


def load_pose_file(path: Path) -> dict:
    """
    Read pose file -> dict {"P0": angle, ...}
    Support:
      - JSON: {"P0": -3, ...} or {"0": -3, ...}
      - Text: P0: -3 / P1 = 89 ...
    """
    if not path.exists():
        print(f"[WARN] Pose file not found: {path}")
        return {}

    txt = path.read_text(encoding="utf-8", errors="ignore").strip()
    if not txt:
        print(f"[WARN] Pose file empty: {path}")
        return {}

    # JSON first
    try:
        obj = json.loads(txt)
        out = {}
        for k, v in obj.items():
            m = re.search(r"(\d+)", str(k))
            if m:
                out[f"P{int(m.group(1))}"] = float(v)
        return out
    except Exception:
        pass

    # Text parse
    out = {}
    for line in txt.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue

        m = re.search(r"[Pp]\s*(\d+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line)
        if m:
            out[f"P{int(m.group(1))}"] = float(m.group(2))
            continue

        # fallback "0 -3"
        nums = re.findall(r"-?\d+(?:\.\d+)?", line)
        if len(nums) >= 2:
            ch = int(float(nums[0]))
            ang = float(nums[1])
            out[f"P{ch}"] = ang

    return out


class SmoothPoseRunner:
    def __init__(self):
        self.ports_all = [f"P{i}" for i in range(12)]
        self.servos = {}
        for p in self.ports_all:
            try:
                self.servos[p] = Servo(p)
            except Exception as e:
                print(f"[WARN] Cannot init Servo({p}): {e}")

        self.current = {p: 0.0 for p in self.ports_all}

    def _move_ports_smooth(self, target: dict, move_ports: list[str], steps=MOVE_STEPS, frame_delay=FRAME_DELAY):
        for i in range(1, steps + 1):
            t = i / steps
            for port in move_ports:
                if port not in target:
                    continue
                if port not in self.servos:
                    continue
                a0 = self.current.get(port, 0.0)
                a1 = float(target[port])
                ang = clamp(lerp(a0, a1, t))
                self.servos[port].angle(ang)
            sleep(frame_delay)

        for port in move_ports:
            if port in target:
                self.current[port] = float(target[port])

    def go_to_in_phases(self, target: dict, name="POSE", settle_sec=SETTLE_SEC, skip_ports=None):
    if skip_ports is None:
        skip_ports = set()
    else:
        skip_ports = set(skip_ports)

    print(f"\n=== MOVE -> {name} ===")
    for phase in PHASES:
        if phase == ["__DELAY1__"]:
            print(f"[PHASE] delay {INTER_PHASE_DELAY_1:.1f}s ...")
            time.sleep(INTER_PHASE_DELAY_1)
            continue

        # chỉ move những port có trong target và KHÔNG nằm trong skip_ports
        move_ports = [p for p in phase if (p in target and p not in skip_ports)]
        if not move_ports:
            continue

        print(f"[PHASE] moving {move_ports} ...")
        self._move_ports_smooth(target, move_ports)

    if settle_sec and settle_sec > 0:
        print(f"[STABLE] settle {settle_sec:.1f}s ...")
        time.sleep(settle_sec)



def main():
    runner = SmoothPoseRunner()

    # 1) PREPOSE theo file config
    pose_cfg = load_pose_file(POSE_FILE)
    if not pose_cfg:
        print("[ERROR] Không đọc được pose config -> dừng.")
        return

    runner.go_to_in_phases(
    pose_cfg,
    name="CONFIG FILE POSE",
    settle_sec=0.0,
    skip_ports={"P4", "P5"}
)

    # 2) delay 1s theo yêu cầu
    print("\n[STEP] Delay 1s after config pose...")
    time.sleep(1.0)

    # 3) -> Pose A (nhẹ)
    runner.go_to_in_phases(POSE_A, name="POSE A", settle_sec=SETTLE_SEC)

    # 4) -> Pose B (nhẹ)
    runner.go_to_in_phases(POSE_B, name="POSE B", settle_sec=SETTLE_SEC)

    print("\n[DONE] Hoàn tất (chưa dùng pidog).")


if __name__ == "__main__":
    main()
