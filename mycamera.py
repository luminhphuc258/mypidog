#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
camera_web_test.py
- Test Picamera2 có trả frame không
- Stream MJPEG lên web:
  http://<PI_IP>:8080  (trang xem)
  http://<PI_IP>:8080/mjpeg  (raw mjpeg)
"""

import time
import io
import threading
import subprocess

from flask import Flask, Response

# Picamera2
from picamera2 import Picamera2

# Encode JPEG: ưu tiên OpenCV, fallback Pillow
ENCODER = None
try:
    import cv2
    ENCODER = "cv2"
except Exception:
    try:
        from PIL import Image
        ENCODER = "pillow"
    except Exception:
        ENCODER = None


HOST = "0.0.0.0"
PORT = 8080
FPS  = 15  # giảm để nhẹ máy

app = Flask(__name__)

latest_jpeg = None
lock = threading.Lock()
stop_evt = threading.Event()

picam2 = None


def kill_camera_users():
    # nếu có process khác đang chiếm camera thì kill (best-effort)
    subprocess.run(
        ["bash", "-lc", "sudo fuser -k /dev/video* /dev/media* 2>/dev/null || true"],
        check=False
    )
    time.sleep(0.2)


def encode_jpeg(frame_rgb):
    """
    frame_rgb: numpy array RGB (H,W,3)
    return: bytes JPEG
    """
    if ENCODER == "cv2":
        # cv2 cần BGR
        bgr = frame_rgb[:, :, ::-1]
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()

    if ENCODER == "pillow":
        img = Image.fromarray(frame_rgb)
        bio = io.BytesIO()
        img.save(bio, format="JPEG", quality=80)
        return bio.getvalue()

    raise RuntimeError("No JPEG encoder available. Install opencv-python or pillow.")


def capture_loop():
    global latest_jpeg
    period = 1.0 / max(1, FPS)

    print(f"[CAM] capture loop started (fps={FPS}, encoder={ENCODER})")
    while not stop_evt.is_set():
        t0 = time.time()
        try:
            frame = picam2.capture_array()  # RGB array
            jpg = encode_jpeg(frame)
            with lock:
                latest_jpeg = jpg
        except Exception as e:
            print(f"[CAM] capture error: {e}")
            time.sleep(0.2)

        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


@app.get("/")
def index():
    return f"""
    <html>
      <head><title>PiDog Camera MJPEG</title></head>
      <body style="background:#111;color:#eee;font-family:Arial">
        <h2>PiDog Camera MJPEG</h2>
        <p>Stream: <a style="color:#8cf" href="/mjpeg">/mjpeg</a></p>
        <img src="/mjpeg" style="max-width:95vw;border:2px solid #444;border-radius:8px" />
      </body>
    </html>
    """


def mjpeg_generator():
    boundary = b"--frame"
    while not stop_evt.is_set():
        with lock:
            frame = latest_jpeg
        if frame is None:
            time.sleep(0.05)
            continue

        yield boundary + b"\r\n" \
              b"Content-Type: image/jpeg\r\n" \
              b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"

        time.sleep(0.001)


@app.get("/mjpeg")
def mjpeg():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
    global picam2

    print("[SYS] killing other camera users (best-effort)...")
    kill_camera_users()

    if ENCODER is None:
        print("[ERR] Thiếu encoder JPEG. Cài 1 trong 2:")
        print("  sudo apt install -y python3-opencv")
        print("  hoặc: sudo apt install -y python3-pil")
        return

    print("[CAM] init Picamera2...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.3)

    # start capture thread
    th = threading.Thread(target=capture_loop, daemon=True)
    th.start()

    print(f"[WEB] open: http://<PI_IP>:{PORT}/  (or /mjpeg)")
    print("[TIP] lấy IP bằng: hostname -I")

    try:
        app.run(host=HOST, port=PORT, debug=False, threaded=True)
    finally:
        stop_evt.set()
        try:
            picam2.stop()
        except:
            pass


if __name__ == "__main__":
    main()
