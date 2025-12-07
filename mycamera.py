#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time
from flask import Flask, Response

# CSI camera (libcamera)
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

app = Flask(__name__)

class StreamingOutput:
    def __init__(self):
        self.frame = None

    def write(self, buf):
        # MJPEGEncoder sẽ gọi write() liên tục
        self.frame = buf

output = StreamingOutput()

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
        if output.frame is None:
            time.sleep(0.02)
            continue
        frame = output.frame
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")

@app.get("/")
def index():
    return "OK. Open /mjpg"

@app.get("/mjpg")
def mjpg():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

def main():
    print("[CAM] init Picamera2...")
    cam = Picamera2()

    # cấu hình nhẹ, dễ chạy
    config = cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    cam.configure(config)

    encoder = MJPEGEncoder(bitrate=2_000_000)
    cam.start_recording(encoder, FileOutput(output))

    ip = get_local_ip()
    print(f"[WEB] http://{ip}:9000/mjpg")
    print("[WEB] Ctrl+C to stop")
    app.run(host="0.0.0.0", port=9000, debug=False, threaded=True)

if __name__ == "__main__":
    main()
