#!/usr/bin/env python3
import cv2
import time
import threading
from flask import Flask, Response
import argparse

app = Flask(__name__)

latest_jpeg = None
lock = threading.Lock()

def capture_loop(dev, w, h, fps, min_area, near_ratio, quality):
    """
    Capture frames -> detect obstacle -> draw boxes -> encode JPEG -> publish to web.
    """
    global latest_jpeg

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {dev}")

    # cố gắng set MJPG để nhẹ CPU
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # background subtractor
    bgs = cv2.createBackgroundSubtractorKNN(history=200, dist2Threshold=400.0, detectShadows=False)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    # fps đo thực tế
    last_t = time.time()
    fps_smooth = 0.0

    time.sleep(0.2)

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.02)
            continue

        H, W = frame.shape[:2]
        frame_area = float(H * W)

        # --- Detect ---
        fg = bgs.apply(frame)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel, iterations=1)
        fg = cv2.morphologyEx(fg, cv2.MORPH_DILATE, kernel, iterations=2)

        cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > best_area:
                best_area = a
                best = c

        label = "clear"
        if best is not None and best_area >= min_area:
            x, y, ww, hh = cv2.boundingRect(best)
            ratio = best_area / frame_area

            if ratio >= near_ratio:
                label = f"OBSTACLE NEAR ({int(ratio*100)}%)"
                cv2.rectangle(frame, (x, y), (x + ww, y + hh), (0, 0, 255), 3)  # đỏ
            else:
                label = f"moving/far ({int(ratio*100)}%)"
                cv2.rectangle(frame, (x, y), (x + ww, y + hh), (0, 255, 255), 2)  # vàng

        # FPS overlay
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
        <title>Robot Camera + Obstacle Detect</title>
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
    ap.add_argument("--min_area", type=int, default=6000)
    ap.add_argument("--near_ratio", type=float, default=0.12)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    dev = parse_dev(args.dev)

    t = threading.Thread(
        target=capture_loop,
        args=(dev, args.w, args.h, args.fps, args.min_area, args.near_ratio, args.quality),
        daemon=True
    )
    t.start()

    app.run(host=args.host, port=args.port, threaded=True)
