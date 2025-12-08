#!/usr/bin/env python3
import cv2
import time
import threading
from flask import Flask, Response
import argparse

app = Flask(__name__)

latest_jpeg = None
lock = threading.Lock()

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def capture_loop(dev, w, h, fps,
                 roi_w_ratio, roi_h_ratio, roi_y_ratio,
                 min_area, near_ratio,
                 stable_hits_required,
                 shake_fg_ratio, shake_cooldown_frames,
                 quality):
    """
    - Detect only in center ROI (front of robot)
    - Require N consecutive frames to confirm obstacle
    - Ignore frames when camera shake is detected (too much FG in ROI)
    """
    global latest_jpeg

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {dev}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    bgs = cv2.createBackgroundSubtractorKNN(history=250, dist2Threshold=400.0, detectShadows=False)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    last_t = time.time()
    fps_smooth = 0.0

    stable_hits = 0
    shake_cooldown = 0

    time.sleep(0.2)

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.02)
            continue

        H, W = frame.shape[:2]

        # --- ROI trung tâm (front) ---
        roi_w = int(W * roi_w_ratio)
        roi_h = int(H * roi_h_ratio)

        roi_w = clamp(roi_w, 50, W)
        roi_h = clamp(roi_h, 50, H)

        x0 = (W - roi_w) // 2
        # roi_y_ratio: 0 = top, 0.5 = center, 1 = bottom
        y_center = int(H * roi_y_ratio)
        y0 = clamp(y_center - roi_h // 2, 0, H - roi_h)

        roi = frame[y0:y0 + roi_h, x0:x0 + roi_w]
        roi_area = float(roi_w * roi_h)

        # --- Background subtraction trong ROI ---
        fg = bgs.apply(roi)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel, iterations=1)
        fg = cv2.morphologyEx(fg, cv2.MORPH_DILATE, kernel, iterations=2)

        # --- “lắc camera” detection ---
        # nếu quá nhiều pixel foreground => camera đang rung/moving mạnh
        fg_nonzero = cv2.countNonZero(fg)
        fg_ratio = fg_nonzero / roi_area

        shaking = (fg_ratio >= shake_fg_ratio) or (shake_cooldown > 0)
        if fg_ratio >= shake_fg_ratio:
            shake_cooldown = shake_cooldown_frames
        elif shake_cooldown > 0:
            shake_cooldown -= 1

        obstacle_now = False
        best_box = None
        best_area = 0

        if not shaking:
            cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                a = cv2.contourArea(c)
                if a > best_area:
                    best_area = a
                    best_box = cv2.boundingRect(c)

            if best_area >= min_area:
                ratio = best_area / roi_area
                obstacle_now = (ratio >= near_ratio)

        # --- ổn định theo thời gian (N frame liên tiếp) ---
        if shaking:
            stable_hits = 0
        else:
            if obstacle_now:
                stable_hits += 1
            else:
                stable_hits = 0

        obstacle_confirmed = (stable_hits >= stable_hits_required)

        # --- Vẽ overlay ---
        # vẽ ROI
        cv2.rectangle(frame, (x0, y0), (x0 + roi_w, y0 + roi_h), (80, 80, 80), 2)

        label = "clear"
        if shaking:
            label = f"SHAKE/FAST MOVE (fg={fg_ratio:.2f})"
        else:
            if best_box is not None and best_area >= min_area:
                bx, by, bw, bh = best_box
                # tọa độ về frame gốc
                gx, gy = x0 + bx, y0 + by

                # màu box: đỏ nếu confirmed, vàng nếu đang “candidate”
                if obstacle_confirmed:
                    cv2.rectangle(frame, (gx, gy), (gx + bw, gy + bh), (0, 0, 255), 3)
                    label = f"OBSTACLE CONFIRMED ({stable_hits}/{stable_hits_required})"
                elif obstacle_now:
                    cv2.rectangle(frame, (gx, gy), (gx + bw, gy + bh), (0, 255, 255), 2)
                    label = f"candidate ({stable_hits}/{stable_hits_required})"
                else:
                    label = "clear"
            else:
                label = "clear"

        # FPS
        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        cv2.putText(frame, f"{label} | FPS: {fps_smooth:.1f}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (255, 255, 255), 2)

        # encode JPEG
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            continue

        with lock:
            latest_jpeg = buf.tobytes()

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

@app.get("/")
def index():
    return """
    <html>
      <head>
        <title>Robot Camera + ROI Obstacle Detect</title>
        <meta name="viewport" content="width=device-width, initial-scale=1" />
      </head>
      <body style="margin:0;background:#111;display:flex;align-items:center;justify-content:center;height:100vh;">
        <img src="/mjpeg" style="max-width:100%;max-height:100%;border:4px solid #333;border-radius:12px;" />
      </body>
    </html>
    """

@app.get("/mjpeg")
def mjpeg():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

def parse_dev(s):
    return int(s) if s.isdigit() else s

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)

    # ROI: rộng, cao, và vị trí theo trục Y (0=trên, 1=dưới)
    ap.add_argument("--roi_w", type=float, default=0.55, help="ROI width ratio")
    ap.add_argument("--roi_h", type=float, default=0.55, help="ROI height ratio")
    ap.add_argument("--roi_y", type=float, default=0.55, help="ROI center Y ratio (0..1)")

    # detect thresholds
    ap.add_argument("--min_area", type=int, default=3500, help="min contour area in ROI")
    ap.add_argument("--near_ratio", type=float, default=0.12, help="area/roi_area to call near")

    # stability
    ap.add_argument("--stable", type=int, default=5, help="frames required to confirm obstacle")

    # shake ignore
    ap.add_argument("--shake_fg", type=float, default=0.60, help="if fg_ratio >= this => shaking")
    ap.add_argument("--shake_cd", type=int, default=6, help="cooldown frames after shake")

    ap.add_argument("--quality", type=int, default=80)
    args = ap.parse_args()

    dev = parse_dev(args.dev)

    t = threading.Thread(
        target=capture_loop,
        args=(dev, args.w, args.h, args.fps,
              args.roi_w, args.roi_h, args.roi_y,
              args.min_area, args.near_ratio,
              args.stable,
              args.shake_fg, args.shake_cd,
              args.quality),
        daemon=True
    )
    t.start()

    app.run(host=args.host, port=args.port, threaded=True)


# obot chạy rung nhiều, hay hiện “SHAKE” quá nhiều → tăng ngưỡng shake:

# --shake_fg 0.75

#python3 cam_web_obstacle_roi_stable.py --w 640 --h 480 --fps 30 --port 8000 --stable 5 --roi_w 0.5 --roi_h 0.55 --roi_y 0.6

# Khoanh đỏ ít quá → giảm --near_ratio hoặc giảm --min_area:

# --near_ratio 0.09 --min_area 2500


# Khoanh bừa nhiều → tăng --stable (7) hoặc tăng --near_ratio:

# --stable 7 --near_ratio 0.15