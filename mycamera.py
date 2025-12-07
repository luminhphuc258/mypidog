#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import socket
import threading
from flask import Flask, Response

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder

app = Flask(__name__)

latest = {"frame": None}
lock = threading.Lock()

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

def mjpeg_generator():
    while True:
        with lock:
            frame = latest["frame"]
        if frame is None:
            time.sleep(0.02)
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")

@app.get("/")
def index():
    return "OK. Open /mjpg"

@app.get("/mjpg")
def mjpg():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

def main():
    print("[CAM] init Picamera2...")
    cam = Picamera2()
    config = cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    cam.configure(config)
    cam.start()

    encoder = MJPEGEncoder(bitrate=2_000_000)

    # callback được gọi mỗi khi encoder tạo ra 1 JPEG frame
    def on_jpeg(buf):
        with lock:
            latest["frame"] = bytes(buf)

    cam.start_recording(encoder, on_jpeg)

    ip = get_local_ip()
    print(f"[WEB] http://{ip}:9000/mjpg")
    print("[WEB] Ctrl+C to stop")
    app.run(host="0.0.0.0", port=9000, debug=False, threaded=True)

if __name__ == "__main__":
    main()
