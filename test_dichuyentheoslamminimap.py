#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import math
import json
import random
import threading
import numpy as np
import serial
from pathlib import Path
from flask import Flask, Response
from smbus2 import SMBus

from robot_hat import Servo
from matthewpidogclassinit import MatthewPidogBootClass
from pidog.preset_actions import bark


# =========================
# CAMERA + WEB CONFIG
# =========================
CAM_DEV = "/dev/video0"
W, H, FPS = 640, 480, 30
WEB_HOST, WEB_PORT = "0.0.0.0", 8000
JPEG_QUALITY = 80

# Edge detection tuned for legs/floor objects
CANNY1, CANNY2 = 50, 150
DILATE_ITER = 2
BLUR_K = (5, 5)

# Ground-plane mask (runs on full frame, but focuses on floor)
TRAP_Y_TOP = 0.45
TRAP_TOP_RATIO = 0.55
TRAP_BOTTOM_RATIO = 1.00

# Contour filters (legs = tall + thin-ish)
MIN_AREA = 300
MIN_H_RATIO = 0.18
MIN_ASPECT = 1.6
NEAR_BOTTOM = 0.80

# "Near < ~50cm" estimation thresholds (tune!)
NEAR_AREA_RATIO = 0.060
NEAR_H_RATIO = 0.28
NEAR_BOTTOM_STRICT = 0.88

# Temporal confirmation
STABLE_N = 5
IOU_MATCH = 0.30
TRACK_TTL_SEC = 0.7

# Special rules
HUGE_COVER_RATIO = 0.70       # if any obstacle bbox covers >70% frame => BACK + LEFT
MULTI_NEAR_DANGER_N = 2       # >=2 near obstacles => PURPLE + TURN_RIGHT_NOW

# Camera blocked / very close detector (full frame, not floor mask)
BLUR_TH = 35.0                # Laplacian variance: low = blurred/close/blocked
EDGE_LOW_TH = 0.010           # edge density low => blocked/blurred
BLOCK_BOX_TH = 0.45           # biggest bbox cover in full frame >45% => blocked

# =========================
# IMU (SH3001)
# =========================
I2C_BUS = 1
ADDR_ACC = 0x36
IMU_HZ = 50
BUMP_HOLD_SEC = 1.2
DECISION_HOLD_SEC = 0.35

# =========================
# UART (ESP32)
# =========================
SERIAL_PORT = "/dev/ttyUSB0"   # kiểm tra bằng `ls /dev/ttyUSB*`
BAUD_RATE = 115200
SAFE_DIST_CM = 50.0           # dưới 50cm coi như gần vật cản

# =========================
# SECTOR / MAP 2D
# =========================
SECTOR_N = 9                  # chia bề ngang FOV thành 9 sector
MAP_H = 80                    # chiều cao mini map (số dòng lịch sử)
MAP_W = SECTOR_N              # chiều rộng = số sector

# =========================
# PIDOG POSE / SERVO CONFIG
# =========================
POSE_FILE = Path(__file__).resolve().parent / "pidog_pose_config.txt"
SERVO_PORTS = [f"P{i}" for i in range(12)]
DELAY_BETWEEN_WRITES = 0.01
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
        print(f"[WARN] Pose file not found: {path} -> all zeros.")
        return cfg
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            for k, v in data.items():
                if k in cfg:
                    cfg[k] = clamp(v)
    except Exception as e:
        print(f"[WARN] Pose file parse error: {e} -> all zeros.")
    return cfg


def apply_pose_config(cfg: dict, step_delay=DELAY_BETWEEN_WRITES, settle_sec=SETTLE_SEC):
    print("[POSE] Apply pose from config file (robot_hat.Servo)...")
    servos = {}
    for p in SERVO_PORTS:
        try:
            servos[p] = Servo(p)
        except Exception as e:
            print(f"[WARN] Cannot init Servo({p}): {e}")

    for p in SERVO_PORTS:
        if p not in servos:
            continue
        try:
            servo_set_angle(servos[p], cfg.get(p, 0))
            time.sleep(step_delay)
        except Exception as e:
            print(f"[WARN] Apply {p} failed: {e}")

    if settle_sec and settle_sec > 0:
        print(f"[POSE] settle {settle_sec:.1f}s ...")
        time.sleep(settle_sec)


# =========================
# HEAD LOCK + WIGGLE
# =========================
def start_head_controller(
    p8_fixed=32,
    p9_fixed=-90,
    p10_a=-70, p10_b=-89,
    write_interval=0.10,
    hold_range=(0.5, 1.4)
):
    """
    Khóa đầu + cổ:
      - P8 luôn p8_fixed
      - P9 luôn p9_fixed
      - P10 lắc nhẹ giữa 2 góc
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
        try:
            s8.angle(clamp(p8_fixed))
            s9.angle(clamp(p9_fixed))
            s10.angle(clamp(p10_b))
        except:
            pass

        target = p10_b
        next_flip = time.time() + random.uniform(*hold_range)

        while not stop_evt.is_set():
            now = time.time()
            if now >= next_flip:
                target = p10_a if target == p10_b else p10_b
                next_flip = now + random.uniform(*hold_range)

            try:
                s8.angle(clamp(p8_fixed))
                s9.angle(clamp(p9_fixed))
                s10.angle(clamp(target))
            except:
                pass

            time.sleep(write_interval)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return stop_evt, t


# =========================
# GLOBAL STATE
# =========================
app = Flask(__name__)
latest_jpeg = None
jpeg_lock = threading.Lock()

# IMU state
bus = SMBus(I2C_BUS)
decision = "CLEAR"
decision_until = 0.0
bump_flag_until = 0.0
last_imu_score = 0.0

# UART state
uart_ready = False
last_uart_line = ""
ultrasonic_cm = None
uart_thread_error = None

# Mini map state (time-history of sectors)
mini_map = np.zeros((MAP_H, MAP_W, 3), dtype=np.uint8)  # BGR

# share sector states between camera + nav
current_sector_states = ["unknown"] * SECTOR_N
sector_lock = threading.Lock()


# =========================
# WEB
# =========================
@app.get("/")
def index():
    return """
    <html><head><title>PiDog Live - Auto Nav + 2D Map</title>
    <meta name="viewport" content="width=device-width,initial-scale=1"/></head>
    <body style="margin:0;background:#111;display:flex;align-items:center;justify-content:center;height:100vh;">
      <img src="/mjpeg" style="max-width:100%;max-height:100%;border:4px solid #333;border-radius:12px;" />
    </body></html>
    """


@app.get("/mjpeg")
def mjpeg():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


def mjpeg_generator():
    while True:
        with jpeg_lock:
            data = latest_jpeg
        if data is None:
            time.sleep(0.03)
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n" +
               data + b"\r\n")
        time.sleep(0.001)


# =========================
# IMU / BUMP
# =========================
def read_word(addr, reg):
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg + 1)
    val = (hi << 8) | lo
    if val & 0x8000:
        val -= 65536
    return val


def read_accel_raw():
    ax = read_word(ADDR_ACC, 0x01)
    ay = read_word(ADDR_ACC, 0x03)
    az = read_word(ADDR_ACC, 0x05)
    return ax, ay, az


def read_gyro_raw():
    gx = read_word(ADDR_ACC, 0x07)
    gy = read_word(ADDR_ACC, 0x09)
    gz = read_word(ADDR_ACC, 0x0B)
    return gx, gy, gz


class BumpDetector:
    def __init__(self):
        self.prev_a = None
        self.ema_ax = None
        self.ema_ay = None
        self.ema_az = None
        self.hits = 0
        self.last_score = 0.0

        self.JERK_TH = 6500
        self.AXIS_DELTA_TH = 4500
        self.GYRO_TH = 12000
        self.CONFIRM_N = 2
        self.ALPHA = 0.06

    def update(self, a, g):
        ax, ay, az = a
        gx, gy, gz = g

        if self.prev_a is None:
            self.prev_a = a
            self.ema_ax, self.ema_ay, self.ema_az = ax, ay, az
            self.last_score = 0.0
            return False

        pax, pay, paz = self.prev_a
        dx, dy, dz = ax - pax, ay - pay, az - paz
        jerk = math.sqrt(dx*dx + dy*dy + dz*dz)
        self.prev_a = a

        self.ema_ax = (1 - self.ALPHA) * self.ema_ax + self.ALPHA * ax
        self.ema_ay = (1 - self.ALPHA) * self.ema_ay + self.ALPHA * ay
        self.ema_az = (1 - self.ALPHA) * self.ema_az + self.ALPHA * az
        dax = abs(ax - self.ema_ax)
        day = abs(ay - self.ema_ay)
        daz = abs(az - self.ema_az)
        axis_delta = max(dax, day, daz)

        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)

        hit = (jerk > self.JERK_TH) or (axis_delta > self.AXIS_DELTA_TH) or (gyro_mag > self.GYRO_TH)
        self.last_score = max(jerk, axis_delta, gyro_mag)

        if hit:
            self.hits += 1
        else:
            self.hits = 0

        if self.hits >= self.CONFIRM_N:
            self.hits = 0
            return True
        return False


bump_detector = BumpDetector()


def set_decision(msg, hold=DECISION_HOLD_SEC):
    global decision, decision_until
    now = time.time()
    decision = msg
    decision_until = now + hold


def imu_loop():
    global bump_flag_until, last_imu_score
    dt = 1.0 / IMU_HZ
    while True:
        try:
            a = read_accel_raw()
            g = read_gyro_raw()
            bumped = bump_detector.update(a, g)
            last_imu_score = bump_detector.last_score
            if bumped:
                bump_flag_until = time.time() + BUMP_HOLD_SEC
        except Exception as e:
            print("[IMU] ERR:", e)
        time.sleep(dt)


# =========================
# UART LOOP (ESP32)
# =========================
def uart_loop():
    global uart_ready, last_uart_line, ultrasonic_cm, uart_thread_error
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2.0)
        uart_ready = True
        print(f"[UART] Connected {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        uart_thread_error = f"open error: {e}"
        print("[UART] ERROR:", e)
        return

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            last_uart_line = line
            parts = line.split(",")
            if len(parts) >= 4:
                try:
                    dist_str = parts[3].strip()
                    if dist_str != "":
                        ultrasonic_cm = float(dist_str)
                except ValueError:
                    pass
        except Exception as e:
            uart_thread_error = f"read error: {e}"
            print("[UART] ERROR:", e)
            time.sleep(0.5)


# =========================
# DETECTION / TRACKING
# =========================
def build_trapezoid_mask(h, w, y_top=0.45, top_ratio=0.55, bottom_ratio=1.0):
    mask = np.zeros((h, w), dtype=np.uint8)
    y1 = int(h * y_top)
    y2 = h - 1
    top_w = int(w * top_ratio)
    bot_w = int(w * bottom_ratio)

    x1a = (w - top_w) // 2
    x1b = x1a + top_w
    x2a = (w - bot_w) // 2
    x2b = x2a + bot_w

    pts = np.array([[x2a, y2], [x2b, y2], [x1b, y1], [x1a, y1]], dtype=np.int32)
    cv2.fillPoly(mask, [pts], 255)
    return mask, pts


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    if inter <= 0:
        return 0.0
    ua = aw * ah + bw * bh - inter
    return inter / float(ua)


def estimate_near(box, contour_area, frame_area, H):
    x, y, w, h = box
    bottom = (y + h) / float(H)
    h_ratio = h / float(H)
    a_ratio = contour_area / float(frame_area)

    near = (
        a_ratio >= NEAR_AREA_RATIO or
        (bottom >= NEAR_BOTTOM_STRICT and h_ratio >= 0.15) or
        h_ratio >= NEAR_H_RATIO
    )
    return near, a_ratio, h_ratio, bottom


class Track:
    def __init__(self, box, near, bbox_ratio):
        self.box = box
        self.near = near
        self.bbox_ratio = bbox_ratio
        self.hits = 1
        self.last_seen = time.time()

    @property
    def confirmed(self):
        return self.hits >= STABLE_N


tracks = []


def update_tracks(detections):
    global tracks
    now = time.time()
    tracks = [t for t in tracks if (now - t.last_seen) <= TRACK_TTL_SEC]

    for det in detections:
        best_i, best_score = -1, 0.0
        for i, t in enumerate(tracks):
            score = iou(det["box"], t.box)
            if score > best_score:
                best_score = score
                best_i = i
        if best_score >= IOU_MATCH and best_i >= 0:
            t = tracks[best_i]
            t.box = det["box"]
            t.near = det["near"]
            t.bbox_ratio = det["bbox_ratio"]
            t.hits = min(t.hits + 1, STABLE_N + 5)
            t.last_seen = now
        else:
            tracks.append(Track(det["box"], det["near"], det["bbox_ratio"]))
    return tracks


# =========================
# SECTOR STATE + MINI MAP
# =========================
def compute_sector_states(W, H, tracks, camera_blocked, ultrasonic_cm):
    sector_states = ["unknown"] * SECTOR_N

    if camera_blocked:
        return ["blocked"] * SECTOR_N

    for i in range(SECTOR_N):
        sector_states[i] = "free"

    for t in tracks:
        if not t.near:
            continue
        x, y, w, h = t.box
        cx = x + w / 2.0
        idx = int(cx / float(W) * SECTOR_N)
        idx = max(0, min(SECTOR_N - 1, idx))
        sector_states[idx] = "blocked"

    mid = SECTOR_N // 2
    if ultrasonic_cm is not None:
        if ultrasonic_cm < SAFE_DIST_CM:
            for j in range(mid - 1, mid + 2):
                if 0 <= j < SECTOR_N:
                    sector_states[j] = "blocked"
        else:
            for j in range(mid - 1, mid + 2):
                if 0 <= j < SECTOR_N and sector_states[j] != "blocked":
                    sector_states[j] = "free"

    return sector_states


def draw_sector_bar_bottom(frame, sector_states):
    h, w, _ = frame.shape
    bar_h = 28
    y0 = h - bar_h - 4
    y1 = h - 4

    for i, state in enumerate(sector_states):
        x0 = int(i * w / SECTOR_N)
        x1 = int((i + 1) * w / SECTOR_N)

        if state == "blocked":
            color = (0, 0, 255)
        elif state == "free":
            color = (0, 200, 0)
        else:
            color = (80, 80, 80)

        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), color, -1)
        alpha = 0.35
        frame[y0:y1, x0:x1] = cv2.addWeighted(
            overlay[y0:y1, x0:x1], alpha,
            frame[y0:y1, x0:x1], 1 - alpha, 0
        )

    cv2.putText(
        frame,
        "Local sectors: GREEN=SAFE  RED=BLOCKED",
        (10, y0 - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1
    )


def update_mini_map(sector_states):
    global mini_map
    mini_map[:-1, :, :] = mini_map[1:, :, :]
    row = np.zeros((MAP_W, 3), dtype=np.uint8)

    for i, state in enumerate(sector_states):
        if state == "blocked":
            c = (0, 0, 255)       # đỏ
        elif state == "free":
            c = (0, 180, 0)       # xanh
        else:
            c = (50, 50, 50)      # xám
        row[i] = c

    mini_map[-1, :, :] = row


def draw_mini_map(frame):
    h, w, _ = frame.shape
    scale = 3
    map_vis = cv2.resize(mini_map, (MAP_W * scale, MAP_H * scale), interpolation=cv2.INTER_NEAREST)

    mh, mw, _ = map_vis.shape
    x1 = w - mw - 8
    y1 = 8
    x2 = x1 + mw
    y2 = y1 + mh

    cv2.rectangle(frame, (x1 - 4, y1 - 4), (x2 + 4, y2 + 20), (0, 0, 0), -1)
    frame[y1:y2, x1:x2] = map_vis

    cv2.putText(frame, "2D mini-map (time history)", (x1, y2 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


# =========================
# CAMERA LOOP
# =========================
def camera_loop():
    global latest_jpeg, current_sector_states

    cap = cv2.VideoCapture(CAM_DEV, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {CAM_DEV}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask_floor, poly = build_trapezoid_mask(H, W, y_top=TRAP_Y_TOP, top_ratio=TRAP_TOP_RATIO, bottom_ratio=TRAP_BOTTOM_RATIO)
    frame_area = float(W * H)

    last_t = time.time()
    fps_smooth = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.02)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, BLUR_K, 0)

        # FULL FRAME BLOCK DETECTOR
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        full_edges = cv2.Canny(gray, CANNY1, CANNY2)
        full_edges = cv2.dilate(full_edges, kernel, iterations=2)
        edge_ratio_full = float(np.count_nonzero(full_edges)) / frame_area

        cnts_full, _ = cv2.findContours(full_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_bbox_cover_full = 0.0
        big_box = None
        for c in cnts_full:
            x, y, w_, h_ = cv2.boundingRect(c)
            cover = (w_ * h_) / frame_area
            if cover > max_bbox_cover_full:
                max_bbox_cover_full = cover
                big_box = (x, y, w_, h_)

        camera_blocked = (max_bbox_cover_full >= BLOCK_BOX_TH) or (lap_var < BLUR_TH and edge_ratio_full < EDGE_LOW_TH)

        # FLOOR OBSTACLE DETECTOR
        edges = cv2.Canny(gray, CANNY1, CANNY2)
        edges = cv2.bitwise_and(edges, edges, mask=mask_floor)
        if DILATE_ITER > 0:
            edges = cv2.dilate(edges, kernel, iterations=DILATE_ITER)

        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < MIN_AREA:
                continue
            x, y, w_, h_ = cv2.boundingRect(c)

            h_ratio = h_ / float(H)
            aspect = h_ / float(max(1, w_))
            bottom = (y + h_) / float(H)

            if h_ratio < MIN_H_RATIO:
                continue
            if aspect < MIN_ASPECT:
                continue
            if bottom < NEAR_BOTTOM:
                continue

            near, _, _, _ = estimate_near((x, y, w_, h_), a, frame_area, H)
            bbox_ratio = (w_ * h_) / frame_area

            detections.append({
                "box": (x, y, w_, h_),
                "near": near,
                "bbox_ratio": bbox_ratio,
            })

        ts = update_tracks(detections)
        near_tracks = [t for t in ts if t.near]
        far_tracks = [t for t in ts if not t.near]

        max_cover = max((t.bbox_ratio for t in ts), default=0.0)

        now = time.time()

        # DECISION TEXT (debug)
        if camera_blocked:
            set_decision("CAMERA_BLOCKED", hold=0.9)
        elif max_cover >= HUGE_COVER_RATIO:
            set_decision("HUGE_OBSTACLE", hold=0.8)
        elif len(near_tracks) >= MULTI_NEAR_DANGER_N:
            set_decision("MULTI_NEAR", hold=0.6)
        elif now < bump_flag_until:
            set_decision("BUMP!", hold=0.6)
        elif len(near_tracks) == 1:
            t = near_tracks[0]
            x, y, w_, h_ = t.box
            cx = x + w_ / 2.0
            if cx < W * 0.5:
                set_decision("NEAR_LEFT", hold=0.35)
            else:
                set_decision("NEAR_RIGHT", hold=0.35)
        else:
            if now > decision_until:
                set_decision("CLEAR", hold=0.25)

        # DRAW OBSTACLES
        cv2.polylines(frame, [poly], True, (120, 120, 120), 2)
        multi_danger = (len(near_tracks) >= MULTI_NEAR_DANGER_N)

        for t in ts:
            x, y, w_, h_ = t.box
            if t.near:
                if multi_danger:
                    color = (255, 0, 255)
                    thick = 3
                else:
                    color = (0, 0, 255) if t.confirmed else (0, 140, 255)
                    thick = 3 if t.confirmed else 2
            else:
                color = (0, 255, 255)
                thick = 2
            cv2.rectangle(frame, (x, y), (x + w_, y + h_), color, thick)

        if camera_blocked and big_box is not None:
            x, y, w_, h_ = big_box
            cv2.rectangle(frame, (x, y), (x + w_, y + h_), (0, 0, 255), 4)

        # SECTOR STATES + MINI MAP
        sector_states = compute_sector_states(W, H, ts, camera_blocked, ultrasonic_cm)

        # lưu lại cho nav loop
        global current_sector_states
        with sector_lock:
            current_sector_states = list(sector_states)

        draw_sector_bar_bottom(frame, sector_states)
        update_mini_map(sector_states)
        draw_mini_map(frame)

        # HUD
        now2 = time.time()
        dt = now2 - last_t
        last_t = now2
        if dt > 0:
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        hud1 = f"DECISION: {decision}"
        hud2 = f"Near:{len(near_tracks)} Far:{len(far_tracks)} maxCover:{max_cover*100:.0f}%"
        hud3 = f"IMU score:{last_imu_score:.0f} bumpHold:{max(0.0, bump_flag_until-time.time()):.1f}s FPS:{fps_smooth:.1f}"

        if uart_thread_error:
            uart_info = f"UART ERR: {uart_thread_error}"
        else:
            dist_txt = f"{ultrasonic_cm:.1f}cm" if ultrasonic_cm is not None else "None"
            uart_info = f"UART ok:{uart_ready} dist:{dist_txt}"

        hud4 = f"block? {camera_blocked} lapVar:{lap_var:.1f} edgeFull:{edge_ratio_full*100:.2f}% boxFull:{max_bbox_cover_full*100:.0f}%"
        hud5 = f"{uart_info} | '{(last_uart_line or '')[:40]}'"

        cv2.putText(frame, hud1, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
        cv2.putText(frame, hud2, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(frame, hud3, (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, hud4, (10, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)
        cv2.putText(frame, hud5, (10, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 1)

        ok2, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok2:
            with jpeg_lock:
                latest_jpeg = buf.tobytes()


# =========================
# NAV LOOP: AUTO FORWARD + AVOID
# =========================
def nav_auto_loop(dog):
    """
    Dùng:
      - current_sector_states (camera)
      - ultrasonic_cm (ESP32)
    để tự quyết định:
      - FORWARD
      - TURN_LEFT / TURN_RIGHT
      - BACK khi bít đường.

    Khi "near obstacle" -> bật đèn đỏ + bark 1 nhịp (cooldown).
    """
    FORWARD_SPEED = 180
    TURN_SPEED = 200
    BACK_SPEED = 200
    STEP_SLEEP = 0.05

    ALERT_COOLDOWN = 2.0
    last_alert_time = 0.0

    try:
        dog.rgb_strip.set_mode("breath", "white", bps=0.6)
    except Exception as e:
        print("[NAV] rgb_strip init error:", e)

    print("[NAV] start AUTO nav loop...")

    while True:
        try:
            with sector_lock:
                sectors = list(current_sector_states)
            dist = ultrasonic_cm

            n = len(sectors)
            if n == 0:
                sectors = ["unknown"] * SECTOR_N
                n = SECTOR_N

            mid = n // 2
            left_sectors = sectors[:max(0, mid - 1)]
            center_sectors = sectors[max(0, mid - 1):min(n, mid + 2)]
            right_sectors = sectors[min(n, mid + 2):]

            blocked_center = any(s == "blocked" for s in center_sectors)
            free_left = any(s == "free" for s in left_sectors)
            free_right = any(s == "free" for s in right_sectors)

            near_by_dist = (dist is not None and dist < SAFE_DIST_CM)
            near_obstacle = blocked_center or near_by_dist

            now = time.time()

            # Báo hiệu LED + sủa
            if near_obstacle:
                try:
                    dog.rgb_strip.set_mode("flash", "red", bps=3)
                except Exception:
                    pass

                if now - last_alert_time > ALERT_COOLDOWN:
                    try:
                        bark(dog, [0, 0, -40])
                    except Exception:
                        pass
                    last_alert_time = now
            else:
                try:
                    dog.rgb_strip.set_mode("breath", "white", bps=0.6)
                except Exception:
                    pass

            # Logic điều hướng
            if near_obstacle and not free_left and not free_right:
                print("[NAV] BLOCKED all sides -> BACK a bit")
                dog.do_action("backward", speed=BACK_SPEED)
                dog.wait_all_done()
                if random.random() < 0.5:
                    dog.do_action("turn_left", step_count=1, speed=TURN_SPEED)
                else:
                    dog.do_action("turn_right", step_count=1, speed=TURN_SPEED)
                dog.wait_all_done()
                time.sleep(STEP_SLEEP)
                continue

            if near_obstacle:
                if free_left and not free_right:
                    print("[NAV] NEAR obstacle -> TURN LEFT")
                    dog.do_action("turn_left", step_count=1, speed=TURN_SPEED)
                    dog.wait_all_done()
                elif free_right and not free_left:
                    print("[NAV] NEAR obstacle -> TURN RIGHT")
                    dog.do_action("turn_right", step_count=1, speed=TURN_SPEED)
                    dog.wait_all_done()
                else:
                    left_free_count = sum(1 for s in left_sectors if s == "free")
                    right_free_count = sum(1 for s in right_sectors if s == "free")
                    if right_free_count > left_free_count:
                        print("[NAV] NEAR -> TURN RIGHT (more free on right)")
                        dog.do_action("turn_right", step_count=1, speed=TURN_SPEED)
                    else:
                        print("[NAV] NEAR -> TURN LEFT (more free on left)")
                        dog.do_action("turn_left", step_count=1, speed=TURN_SPEED)
                    dog.wait_all_done()

                time.sleep(STEP_SLEEP)
                continue

            # Không near_obstacle -> đi thẳng chậm
            print("[NAV] CLEAR -> FORWARD")
            dog.do_action("forward", speed=FORWARD_SPEED)
            dog.wait_all_done()
            time.sleep(STEP_SLEEP)

        except Exception as e:
            print("[NAV] ERR:", e)
            time.sleep(0.2)


# =========================
# MAIN
# =========================
def main():
    print("=== Pidog Auto Nav + Camera + IMU + UART + MiniMap ===")

    # Step 1: set pose từ file
    cfg = load_pose_config(POSE_FILE)
    apply_pose_config(cfg, step_delay=DELAY_BETWEEN_WRITES, settle_sec=1.0)

    # Step 2: boot Pidog qua MatthewPidogBootClass
    print("[BOOT] MatthewPidogBootClass...")
    boot = MatthewPidogBootClass()
    dog = boot.create()
    time.sleep(1.0)

    # Đưa về tư thế đứng
    dog.do_action("stand", speed=10)
    dog.wait_all_done()
    bark(dog, [0, 0, -40])
    time.sleep(0.3)

    # Khóa đầu
    head_stop_evt, head_thread = start_head_controller(
        p8_fixed=32,
        p9_fixed=-90,
        p10_a=-70,
        p10_b=-90,
        write_interval=0.08,
        hold_range=(0.6, 1.6)
    )

    # Thread IMU, UART, CAMERA, NAV
    threading.Thread(target=imu_loop, daemon=True).start()
    threading.Thread(target=uart_loop, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()
    threading.Thread(target=nav_auto_loop, args=(dog,), daemon=True).start()

    try:
        app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)
    finally:
        if head_stop_evt is not None:
            head_stop_evt.set()
        if head_thread is not None:
            head_thread.join(timeout=0.5)
        print("[EXIT] Clean up done.")


if __name__ == "__main__":
    main()
