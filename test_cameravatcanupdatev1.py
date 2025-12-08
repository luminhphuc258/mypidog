#!/usr/bin/env python3
import cv2
import time
import threading
from flask import Flask, Response
import argparse

app = Flask(__name__)
latest_jpeg = None
lock = threading.Lock()

def clamp(v, lo, hi): return max(lo, min(hi, v))
def parse_dev(s): return int(s) if s.isdigit() else s

def capture_loop(dev, w, h, fps,
                 roi_w_ratio, roi_h_ratio, roi_y_ratio,
                 min_area,
                 near_area_ratio,
                 tall_h_ratio, min_w_ratio,
                 stable_hits_required,
                 shake_fg_ratio, shake_cooldown_frames,
                 learn_sec, freeze_bg, lr_slow,
                 quality):

    global latest_jpeg

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {dev}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    bgs = cv2.createBackgroundSubtractorKNN(history=300, dist2Threshold=400.0, detectShadows=False)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    start_t = time.time()
    last_t = start_t
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

        # --- ROI trung tâm ---
        roi_w = clamp(int(W * roi_w_ratio), 80, W)
        roi_h = clamp(int(H * roi_h_ratio), 80, H)

        x0 = (W - roi_w) // 2
        y_center = int(H * roi_y_ratio)
        y0 = clamp(y_center - roi_h // 2, 0, H - roi_h)

        roi = frame[y0:y0+roi_h, x0:x0+roi_w]
        roi_area = float(roi_w * roi_h)

        # --- learningRate control (chìa khóa để không “học mất” vật cản) ---
        elapsed = time.time() - start_t
        if elapsed < learn_sec:
            # học nhanh trong vài giây đầu
            lr = -1  # auto
        else:
            if freeze_bg:
                lr = 0.0  # đóng băng background => vật mới sẽ stay foreground
            else:
                lr = lr_slow  # vẫn học nhưng chậm

        fg = bgs.apply(roi, learningRate=lr)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel, iterations=1)
        fg = cv2.morphologyEx(fg, cv2.MORPH_DILATE, kernel, iterations=2)

        # --- detect rung / fast move ---
        fg_nonzero = cv2.countNonZero(fg)
        fg_ratio = fg_nonzero / roi_area

        if fg_ratio >= shake_fg_ratio:
            shake_cooldown = shake_cooldown_frames
        elif shake_cooldown > 0:
            shake_cooldown -= 1

        shaking = (fg_ratio >= shake_fg_ratio) or (shake_cooldown > 0)

        obstacle_now = False
        best_box = None
        best_area = 0
        best_ratio = 0.0
        tall_ratio = 0.0
        w_ratio = 0.0

        if not shaking:
            cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                a = cv2.contourArea(c)
                if a > best_area:
                    best_area = a
                    best_box = cv2.boundingRect(c)

            if best_box is not None and best_area >= min_area:
                bx, by, bw, bh = best_box
                best_ratio = best_area / roi_area
                tall_ratio = bh / float(roi_h)
                w_ratio = bw / float(roi_w)

                # 2 tiêu chí:
                # - vật to (diện tích)
                # - vật mảnh nhưng cao (chân bàn/ghế)
                obstacle_now = (best_ratio >= near_area_ratio) or (tall_ratio >= tall_h_ratio and w_ratio >= min_w_ratio)

        # --- ổn định N frame liên tiếp ---
        if shaking:
            stable_hits = 0
        else:
            stable_hits = stable_hits + 1 if obstacle_now else 0

        obstacle_confirmed = (stable_hits >= stable_hits_required)

        # --- vẽ overlay ---
        cv2.rectangle(frame, (x0, y0), (x0+roi_w, y0+roi_h), (80, 80, 80), 2)

        label = "clear"
        if shaking:
            label = f"SHAKE (fg={fg_ratio:.2f})"
        else:
            if best_box is not None and best_area >= min_area:
                bx, by, bw, bh = best_box
                gx, gy = x0 + bx, y0 + by

                if obstacle_confirmed:
                    cv2.rectangle(frame, (gx, gy), (gx+bw, gy+bh), (0, 0, 255), 3)
                    label = f"OBSTACLE ({stable_hits}/{stable_hits_required})"
                elif obstacle_now:
                    cv2.rectangle(frame, (gx, gy), (gx+bw, gy+bh), (0, 255, 255), 2)
                    label = f"candidate ({stable_hits}/{stable_hits_required})"
                else:
                    label = "clear"

        # FPS
        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

        # debug nhỏ để tune
        dbg = f"{label} | FPS:{fps_smooth:.1f} | A:{int(best_area)} r:{best_ratio:.2f} h:{tall_ratio:.2f} w:{w_ratio:.2f} lr:{lr}"
        cv2.putText(frame, dbg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
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
    <html><head><title>Robot Cam + ROI Obstacle</title><meta name="viewport" content="width=device-width,initial-scale=1"/></head>
    <body style="margin:0;background:#111;display:flex;align-items:center;justify-content:center;height:100vh;">
      <img src="/mjpeg" style="max-width:100%;max-height:100%;border:4px solid #333;border-radius:12px;" />
    </body></html>
    """

@app.get("/mjpeg")
def mjpeg():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)

    # ROI
    ap.add_argument("--roi_w", type=float, default=0.75)
    ap.add_argument("--roi_h", type=float, default=0.60)
    ap.add_argument("--roi_y", type=float, default=0.65)

    # detect
    ap.add_argument("--min_area", type=int, default=1200)
    ap.add_argument("--near_ratio", type=float, default=0.08)   # diện tích
    ap.add_argument("--tall_h", type=float, default=0.55)       # bắt chân bàn: cao
    ap.add_argument("--min_w", type=float, default=0.02)        # và không quá mỏng

    # stability
    ap.add_argument("--stable", type=int, default=5)

    # shake ignore
    ap.add_argument("--shake_fg", type=float, default=0.70)
    ap.add_argument("--shake_cd", type=int, default=6)

    # background learning
    ap.add_argument("--learn_sec", type=float, default=2.0)
    ap.add_argument("--freeze_bg", action="store_true")         # khuyên bật
    ap.add_argument("--lr_slow", type=float, default=0.01)      # nếu không freeze

    ap.add_argument("--quality", type=int, default=80)
    args = ap.parse_args()

    dev = parse_dev(args.dev)

    t = threading.Thread(
        target=capture_loop,
        args=(dev, args.w, args.h, args.fps,
              args.roi_w, args.roi_h, args.roi_y,
              args.min_area, args.near_ratio,
              args.tall_h, args.min_w,
              args.stable,
              args.shake_fg, args.shake_cd,
              args.learn_sec, args.freeze_bg, args.lr_slow,
              args.quality),
        daemon=True
    )
    t.start()

    app.run(host=args.host, port=args.port, threaded=True)
