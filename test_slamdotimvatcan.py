#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import math
import threading
import numpy as np
import serial
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

# IMU SH3001
I2C_BUS = 1
ADDR_ACC = 0x36

IMU_HZ = 50
BUMP_HOLD_SEC = 1.2           # show bump decision this long

# Decision hold (reduce flicker)
DECISION_HOLD_SEC = 0.35

# ========= NEW: UART từ ESP32 (ultrasonic / obstacle sensor) =========
SERIAL_PORT = "/dev/ttyUSB0"   # kiểm tra bằng: ls /dev/ttyUSB*
BAUD_RATE = 115200

# Đơn vị an toàn cho vùng phía trước (cm)
SAFE_DIST_CM = 50.0

# Số sector (chia khung hình thành N hướng)
SECTOR_N = 9

# =========================
# GLOBAL STATE
# =========================
app = Flask(__name__)
latest_jpeg = None
lock = threading.Lock()

# UART shared state
uart_ready = False
last_uart_line = ""
ultrasonic_cm = None
uart_thread_error = None


# =========================
# WEB MJPEG
# =========================
@app.get("/")
def index():
    return """
    <html><head><title>PiDog Live - Local 2D Map (Camera + IMU + UART)</title>
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
# IMU (SH3001) + Bump detector
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


def read_gyro_raw():
    gx = read_word(ADDR_ACC, 0x07)
    gy = read_word(ADDR_ACC, 0x09)
    gz = read_word(ADDR_ACC, 0x0B)
    return gx, gy, gz


class BumpDetector:
    """
    Improved bump detector:
    - Jerk catches sudden impact
    - Baseline drift (EMA) catches slow push
    - Gyro magnitude catches twist/impact
    """
    def __init__(self):
        self.prev_a = None
        self.ema_ax = None
        self.ema_ay = None
        self.ema_az = None
        self.hits = 0
        self.last_score = 0.0

        # TUNE THESE
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


decision = "GO_STRAIGHT (CLEAR)"
decision_until = 0.0

bump_flag_until = 0.0
bump_detector = BumpDetector()
last_imu_score = 0.0


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
        except Exception:
            # tránh crash nếu IMU lỗi tạm thời
            pass
        time.sleep(dt)


# =========================
# UART LOOP (ESP32 -> Raspberry Pi)
# Format chuỗi (ví dụ từ code Arduino): "timestamp_ms,tempC,humidity,ultrasonic_cm"
# =========================
def uart_loop():
    global uart_ready, last_uart_line, ultrasonic_cm, uart_thread_error
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2.0)  # đợi ESP32 reset
        uart_ready = True
        print(f"[UART] Connected to {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        uart_thread_error = f"UART open error: {e}"
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

            # trường hợp format: timestamp,temp,humidity,ultrasonic_cm
            if len(parts) >= 4:
                try:
                    dist_str = parts[3].strip()
                    if dist_str != "":
                        ultrasonic_cm = float(dist_str)
                except ValueError:
                    # bỏ qua dòng lỗi parse
                    pass

        except Exception as e:
            uart_thread_error = f"UART read error: {e}"
            print("[UART] ERROR:", e)
            time.sleep(0.5)


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


# =========================
# Simple tracker
# =========================
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
# 2D-LIKE LOCAL MAP (SECTOR) FUSION
# =========================
def compute_sector_states(W, H, tracks, camera_blocked, ultrasonic_cm):
    """
    Tạo map đơn giản theo SECTOR_N sector phía trước:
      - 'blocked'  = có near obstacle trong sector hoặc ultrasonic < SAFE_DIST_CM
      - 'free'     = không có vật cản & ultrasonic đủ xa
      - 'unknown'  = không rõ (ít dùng, chủ yếu lúc camera bị lỗi)
    """
    sector_states = ["unknown"] * SECTOR_N

    # nếu camera bị che -> coi như tất cả block
    if camera_blocked:
        return ["blocked"] * SECTOR_N

    # mặc định = free, rồi sẽ gán blocked nếu có obstacle
    for i in range(SECTOR_N):
        sector_states[i] = "free"

    # đánh dấu sector có near obstacle trên hình
    for t in tracks:
        if not t.near:
            continue
        x, y, w, h = t.box
        cx = x + w / 2.0
        sector_idx = int(cx / float(W) * SECTOR_N)
        sector_idx = max(0, min(SECTOR_N - 1, sector_idx))
        sector_states[sector_idx] = "blocked"

    # dùng ultrasonic cho sector giữa (hướng robot đang nhìn thẳng)
    mid = SECTOR_N // 2
    if ultrasonic_cm is not None:
        if ultrasonic_cm < SAFE_DIST_CM:
            # block vùng trung tâm + 1 sector bên cạnh cho “ống” phía trước
            for j in range(mid - 1, mid + 2):
                if 0 <= j < SECTOR_N:
                    sector_states[j] = "blocked"
        else:
            # nếu chưa bị block bởi camera, đánh dấu free rõ cho vùng giữa
            for j in range(mid - 1, mid + 2):
                if 0 <= j < SECTOR_N and sector_states[j] != "blocked":
                    sector_states[j] = "free"

    return sector_states


def draw_sector_overlay(frame, sector_states):
    """
    Vẽ thanh màu ở đáy khung hình: XANH = free, ĐỎ = blocked, XÁM = unknown.
    Coi như bản đồ 2D local phía trước robot (FOV camera).
    """
    h, w, _ = frame.shape
    bar_h = 32
    y0 = h - bar_h - 4
    y1 = h - 4

    for i, state in enumerate(sector_states):
        x0 = int(i * w / SECTOR_N)
        x1 = int((i + 1) * w / SECTOR_N)

        if state == "blocked":
            color = (0, 0, 255)      # RED
        elif state == "free":
            color = (0, 200, 0)      # GREEN
        else:
            color = (80, 80, 80)     # GRAY (unknown)

        # vẽ overlay với alpha
        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), color, -1)
        alpha = 0.35
        frame[y0:y1, x0:x1] = cv2.addWeighted(
            overlay[y0:y1, x0:x1], alpha,
            frame[y0:y1, x0:x1], 1 - alpha, 0
        )

    # text chú thích
    cv2.putText(
        frame,
        "Local 2D Map: GREEN=SAFE  RED=BLOCKED",
        (10, y0 - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2
    )


# =========================
# Camera loop
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

        # ===== FULL FRAME BLOCK DETECTOR =====
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()

        full_edges = cv2.Canny(gray, CANNY1, CANNY2)
        full_edges = cv2.dilate(full_edges, kernel, iterations=2)
        edge_ratio_full = float(np.count_nonzero(full_edges)) / frame_area

        cnts_full, _ = cv2.findContours(full_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_bbox_cover_full = 0.0
        big_box = None
        for c in cnts_full:
            x, y, w, h = cv2.boundingRect(c)
            cover = (w * h) / frame_area
            if cover > max_bbox_cover_full:
                max_bbox_cover_full = cover
                big_box = (x, y, w, h)

        camera_blocked = (max_bbox_cover_full >= BLOCK_BOX_TH) or (lap_var < BLUR_TH and edge_ratio_full < EDGE_LOW_TH)

        # ===== FLOOR EDGE OBSTACLE DETECTOR =====
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

            near, _, _, _ = estimate_near((x, y, w, h), a, frame_area, H)
            bbox_ratio = (w * h) / frame_area  # IMPORTANT: use bbox cover ratio for big object

            detections.append({
                "box": (x, y, w, h),
                "near": near,
                "bbox_ratio": bbox_ratio,
            })

        ts = update_tracks(detections)
        near_tracks = [t for t in ts if t.near]
        far_tracks = [t for t in ts if not t.near]

        # max cover from tracked boxes (bbox ratio)
        max_cover = 0.0
        for t in ts:
            max_cover = max(max_cover, t.bbox_ratio)

        now = time.time()

        # ===== DECISION PRIORITY =====
        if camera_blocked:
            set_decision("CAMERA_BLOCKED -> BACK + TURN_LEFT", hold=0.9)
        elif max_cover >= HUGE_COVER_RATIO:
            set_decision("BACK + TURN_LEFT (HUGE OBSTACLE)", hold=0.8)
        elif len(near_tracks) >= MULTI_NEAR_DANGER_N:
            set_decision("DANGER: MULTI NEAR -> TURN_RIGHT_NOW", hold=0.6)
        elif now < bump_flag_until:
            set_decision("BUMP! -> BACK + TURN_RIGHT", hold=0.6)
        elif len(near_tracks) == 1:
            t = near_tracks[0]
            x, y, w, h = t.box
            cx = x + w / 2.0
            if cx < W * 0.5:
                set_decision("NEAR -> TURN_RIGHT", hold=0.35)
            else:
                set_decision("NEAR -> TURN_LEFT", hold=0.35)
        else:
            if now > decision_until:
                set_decision("GO_STRAIGHT (CLEAR)", hold=0.25)

        # ===== DRAW OVERLAYS (OBSTACLES) =====
        cv2.polylines(frame, [poly], True, (120, 120, 120), 2)

        multi_danger = (len(near_tracks) >= MULTI_NEAR_DANGER_N)

        for t in ts:
            x, y, w, h = t.box
            if t.near:
                if multi_danger:
                    color = (255, 0, 255)      # purple
                    thick = 3
                else:
                    color = (0, 0, 255) if t.confirmed else (0, 140, 255)  # red / orange
                    thick = 3 if t.confirmed else 2
            else:
                color = (0, 255, 255)          # yellow
                thick = 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thick)

        # camera blocked debug box (full-frame)
        if camera_blocked and big_box is not None:
            x, y, w, h = big_box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 4)

        # ===== LOCAL 2D MAP FUSION (SECTOR) =====
        sector_states = compute_sector_states(
            W, H, ts, camera_blocked, ultrasonic_cm
        )
        draw_sector_overlay(frame, sector_states)

        # ===== FPS & HUD =====
        now2 = time.time()
        dt = now2 - last_t
        last_t = now2
        if dt > 0:
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        hud1 = f"DECISION: {decision}"
        hud2 = f"Near:{len(near_tracks)} Far:{len(far_tracks)}  maxCover:{max_cover*100:.0f}%"
        hud3 = f"IMU score:{last_imu_score:.0f}  bumpHold:{max(0.0, bump_flag_until-time.time()):.1f}s  FPS:{fps_smooth:.1f}"

        if uart_thread_error:
            uart_info = f"UART ERR: {uart_thread_error}"
        else:
            dist_txt = f"{ultrasonic_cm:.1f}cm" if ultrasonic_cm is not None else "None"
            uart_info = f"UART ok:{uart_ready}  dist:{dist_txt}"

        hud4 = f"block? {camera_blocked}  lapVar:{lap_var:.1f}  edgeFull:{edge_ratio_full*100:.2f}%  boxFull:{max_bbox_cover_full*100:.0f}%"
        hud5 = f"{uart_info} | raw:'{last_uart_line[:48]}'"

        cv2.putText(frame, hud1, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
        cv2.putText(frame, hud2, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(frame, hud3, (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, hud4, (10, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, hud5, (10, 134), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            with lock:
                latest_jpeg = buf.tobytes()


def start():
    # IMU bump detector
    threading.Thread(target=imu_loop, daemon=True).start()
    # UART ESP32 (ultrasonic + other sensor data)
    threading.Thread(target=uart_loop, daemon=True).start()
    # Camera + local map
    threading.Thread(target=camera_loop, daemon=True).start()

    app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)


if __name__ == "__main__":
    print("Starting web stream + obstacle detection + IMU bump + UART fusion local 2D map...")
    start()
