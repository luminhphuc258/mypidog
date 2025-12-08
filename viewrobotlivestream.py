#!/usr/bin/env python3
import cv2
import time
import threading
from flask import Flask, Response
import argparse

app = Flask(__name__)

latest_jpeg = None
lock = threading.Lock()

def capture_loop(dev, w, h, fps):
    global latest_jpeg
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {dev}")

    # cố gắng set MJPG để nhẹ CPU
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # warm-up
    time.sleep(0.2)

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        # encode JPEG
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            continue

        with lock:
            latest_jpeg = buf.tobytes()

def mjpeg_generator():
    # stream MJPEG multipart
    while True:
        with lock:
            data = latest_jpeg
        if data is None:
            time.sleep(0.05)
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
      <head><title>Robot Camera Live</title></head>
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
    # allow "/dev/video0" or "0"
    if s.isdigit():
        return int(s)
    return s

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    dev = parse_dev(args.dev)

    t = threading.Thread(target=capture_loop, args=(dev, args.w, args.h, args.fps), daemon=True)
    t.start()

    # threaded=True để nhiều client xem cùng lúc
    app.run(host=args.host, port=args.port, threaded=True)
