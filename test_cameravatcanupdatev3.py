#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import math
import threading
import numpy as np
from flask import Flask, Response
from smbus2 import SMBus

# =========================
# CONFIG
# =========================
CAM_DEV = "/dev/video0"
W, H, FPS = 640, 480, 30
WEB_HOST, WEB_PORT = "0.0.0.0", 8000
JPEG_QUALITY = 80

# Edge detection tuned for legs/floor objects
CANNY1, CANNY2 = 50, 150
DILATE_ITER = 2
BLUR_K = (5, 5)

# Ground-plane mask (full frame, not cropping ROI)
# Trapezoid focuses lower area (floor) but still runs on full frame.
TRAP_Y_TOP = 0.45      # start of ground mask (0..1 of frame height)
TRAP_TOP_RATIO = 0.55  # width of trapezoid top edge vs frame width (0..1)
TRAP_BOTTOM_RATIO = 1.00

# Contour filters (legs = tall + thin-ish)
MIN_AREA = 300
MIN_H_RATIO = 0.18     # bbox height / frame height
MIN_ASPECT = 1.6       # bbox height / bbox width
NEAR_BOTTOM = 0.80     # bbox bottom / frame height (must be near bottom)

# "Near < ~50cm" estimation thresholds (tune this!)
# If any of these is true -> treat as NEAR
NEAR_AREA_RATIO = 0.060      # contour area / frame area
NEAR_H_RATIO = 0.28          # bbox height / frame height
NEAR_BOTTOM_STRICT = 0.88    # bbox bottom close to bottom

# Temporal confirmation
STABLE_N = 5
IOU_MATCH = 0.30
TRACK_TTL_SEC = 0.7

# Special rules
HUGE_COVER_RATIO = 0.70  # if any obstacle covers >70% frame => BACK + LEFT
MULTI_NEAR_DANGER_N = 2  # >=2 near obstacles => purple + TURN_RIGHT_NOW

# IMU SH3001 (your tested addresses)
I2C_BUS = 1
ADDR_ACC = 0x36

# IMU bump detection (jerk-based on accel raw)
IMU_HZ = 50
JERK_TH = 9000          # tune based on your logs
BUMP_CONFIRM_N = 2
BUMP_HOLD_SEC = 1.2     # show decision for this duration

# Decision hold for camera decisions (avoid flicker)
DECISION_HOLD_SEC = 0.35


# =========================
# WEB MJPEG
# =========================
app = Flask(__name__)
latest_jpeg = None
lock = threading.Lock()

@app.get("/")
def index():
    return """
    <html><head><title>PiDog Live - Obstacle + IMU Decision</title>
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
        with lock:
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
# IMU (your code reused)
# =========================
bus = SMBus(I2C_BUS)

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


class BumpDetector:
    def __init__(self, jerk_th=9000, confirm_n=2):
        self.jerk_th = jerk_th
        self.confirm_n = confirm_n
        self.prev = None
        self.hits = 0
        self.last_jerk = 0.0

    def update(self, a):
        if self.prev is None:
            self.prev = a
            self.last_jerk = 0.0
            return False

        ax, ay, az = a
        px, py, pz = self.prev
        dx, dy, dz = ax - px, ay - py, az - pz
        jerk = math.sqrt(dx*dx + dy*dy + dz*dz)
        self.last_jerk = jerk
        self.prev = a

        if jerk > self.jerk_th:
            self.hits += 1
        else:
            self.hits = 0

        if self.hits >= self.confirm_n:
            self.hits = 0
            return True
        return False


# =========================
# Detection helpers
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

def estimate_near(box, area, frame_area, H):
    x, y, w, h = box
    bottom = (y + h) / float(H)
    h_ratio = h / float(H)
    a_ratio = area / float(frame_area)

    near = (
        a_ratio >= NEAR_AREA_RATIO or
        (bottom >= NEAR_BOTTOM_STRICT and h_ratio >= 0.15) or
        h_ratio >= NEAR_H_RATIO
    )
    return near, a_ratio, h_ratio, bottom


# =========================
# Simple tracker for confirmation
# =========================
class Track:
    def __init__(self, box, near, area_ratio):
        self.box = box
        self.near = near
        self.area_ratio = area_ratio
        self.hits = 1
        self.last_seen = time.time()

    @property
    def confirmed(self):
        return self.hits >= STABLE_N

tracks = []

def update_tracks(detections):
    """
    detections: list of dict {box, near, area_ratio}
    """
    global tracks
    now = time.time()

    # prune old
    tracks = [t for t in tracks if (now - t.last_seen) <= TRACK_TTL_SEC]

    matched = set()

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
            t.area_ratio = det["area_ratio"]
            t.hits = min(t.hits + 1, STABLE_N + 5)
            t.last_seen = now
            matched.add(best_i)
        else:
            tracks.append(Track(det["box"], det["near"], det["area_ratio"]))

    return tracks


# =========================
# Decision logic (display only)
# =========================
decision = "GO_STRAIGHT"
decision_until = 0.0

bump_flag_until = 0.0
bump_detector = BumpDetector(jerk_th=JERK_TH, confirm_n=BUMP_CONFIRM_N)
last_jerk = 0.0

def set_decision(msg, hold=DECISION_HOLD_SEC):
    global decision, decision_until
    now = time.time()
    decision = msg
    decision_until = now + hold

def imu_loop():
    global bump_flag_until, last_jerk
    dt = 1.0 / IMU_HZ
    while True:
        try:
            a = read_accel_raw()
            bumped = bump_detector.update(a)
            last_jerk = bump_detector.last_jerk
            if bumped:
                bump_flag_until = time.time() + BUMP_HOLD_SEC
        except Exception:
            pass
        time.sleep(dt)


# =========================
# Camera loop: detect + draw + decide + stream
# =========================
def camera_loop():
    global latest_jpeg

    cap = cv2.VideoCapture(CAM_DEV, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {CAM_DEV}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask, poly = build_trapezoid_mask(H, W, y_top=TRAP_Y_TOP, top_ratio=TRAP_TOP_RATIO, bottom_ratio=TRAP_BOTTOM_RATIO)
    frame_area = float(W * H)

    last_t = time.time()
    fps_smooth = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.02)
            continue

        # --- Edge-based obstacle on full frame ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, BLUR_K, 0)

        edges = cv2.Canny(gray, CANNY1, CANNY2)
        edges = cv2.bitwise_and(edges, edges, mask=mask)
        if DILATE_ITER > 0:
            edges = cv2.dilate(edges, kernel, iterations=DILATE_ITER)

        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)

            h_ratio = h / float(H)
            aspect = h / float(max(1, w))
            bottom = (y + h) / float(H)

            if h_ratio < MIN_H_RATIO:
                continue
            if aspect < MIN_ASPECT:
                continue
            if bottom < NEAR_BOTTOM:
                continue

            near, a_ratio, _, _ = estimate_near((x, y, w, h), a, frame_area, H)

            detections.append({
                "box": (x, y, w, h),
                "near": near,
                "area_ratio": a_ratio,
            })

        # update tracks for confirmation
        ts = update_tracks(detections)

        # collect confirmed/current near/far
        near_tracks = [t for t in ts if t.near]
        far_tracks  = [t for t in ts if not t.near]

        # huge cover rule (use max area_ratio of near tracks)
        max_cover = 0.0
        for t in ts:
            max_cover = max(max_cover, t.area_ratio)

        now = time.time()

        # --- Decide (priority rules) ---
        # 1) huge cover >70% => BACK + LEFT
        if max_cover >= HUGE_COVER_RATIO:
            set_decision("BACK + TURN_LEFT (HUGE OBSTACLE)", hold=0.8)

        # 2) multiple near obstacles (<~50cm) => purple + TURN RIGHT NOW
        elif len(near_tracks) >= MULTI_NEAR_DANGER_N:
            set_decision("DANGER: MULTI NEAR -> TURN_RIGHT_NOW", hold=0.6)

        # 3) bump detected => BACK + RIGHT
        elif now < bump_flag_until:
            set_decision("BUMP! -> BACK + TURN_RIGHT", hold=0.6)

        # 4) single near obstacle => choose turn based on obstacle x position
        elif len(near_tracks) == 1:
            t = near_tracks[0]
            x, y, w, h = t.box
            cx = x + w / 2.0
            if cx < W * 0.5:
                set_decision("NEAR -> TURN_RIGHT", hold=0.35)
            else:
                set_decision("NEAR -> TURN_LEFT", hold=0.35)

        # 5) only far obstacles => clear, go straight
        else:
            # keep current decision until timeout to reduce flicker
            if now > decision_until:
                set_decision("GO_STRAIGHT (CLEAR)", hold=0.25)

        # --- Draw overlays ---
        # draw ground trapezoid
        cv2.polylines(frame, [poly], True, (120, 120, 120), 2)

        # coloring rules:
        # - Near (<~50cm): confirmed = RED, unconfirmed = ORANGE-ish (still near)
        # - Far (>~50cm): YELLOW
        # - Multi near danger: NEAR boxes = PURPLE
        multi_danger = (len(near_tracks) >= MULTI_NEAR_DANGER_N)

        for t in ts:
            x, y, w, h = t.box
            if t.near:
                if multi_danger:
                    color = (255, 0, 255)  # purple (BGR)
                    thick = 3
                else:
                    color = (0, 0, 255) if t.confirmed else (0, 140, 255)  # red / orange
                    thick = 3 if t.confirmed else 2
            else:
                color = (0, 255, 255)  # yellow
                thick = 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thick)

        # HUD text
        now2 = time.time()
        dt = now2 - last_t
        last_t = now2
        if dt > 0:
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        hud1 = f"DECISION: {decision}"
        hud2 = f"Near:{len(near_tracks)} Far:{len(far_tracks)}  maxCover:{max_cover*100:.0f}%"
        hud3 = f"IMU jerk:{last_jerk:.0f} th:{JERK_TH}  bumpHold:{max(0.0, bump_flag_until-time.time()):.1f}s  FPS:{fps_smooth:.1f}"

        cv2.putText(frame, hud1, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
        cv2.putText(frame, hud2, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(frame, hud3, (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            with lock:
                latest_jpeg = buf.tobytes()


def start():
    threading.Thread(target=imu_loop, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()
    app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)

if __name__ == "__main__":
    print("Starting web stream + edge obstacle + IMU bump decision...")
    start()
