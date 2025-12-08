#!/usr/bin/env python3
import cv2
import time
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0", help="camera device, e.g. /dev/video0 or 0")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--min_area", type=int, default=6000, help="min contour area to consider")
    ap.add_argument("--near_ratio", type=float, default=0.12, help="area/frame_area to mark as NEAR")
    ap.add_argument("--show_mask", action="store_true")
    args = ap.parse_args()

    dev = int(args.dev) if args.dev.isdigit() else args.dev
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera: {args.dev}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    # Background subtractor: khá ổn cho demo
    bgs = cv2.createBackgroundSubtractorKNN(history=200, dist2Threshold=400.0, detectShadows=False)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    last_t = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed")
            break

        H, W = frame.shape[:2]
        frame_area = float(H * W)

        fg = bgs.apply(frame)

        # lọc nhiễu
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel, iterations=1)
        fg = cv2.morphologyEx(fg, cv2.MORPH_DILATE, kernel, iterations=2)

        cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # tìm contour lớn nhất
        best = None
        best_area = 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > best_area:
                best_area = a
                best = c

        # vẽ kết quả
        label = "clear"
        if best is not None and best_area >= args.min_area:
            x, y, w, h = cv2.boundingRect(best)
            ratio = best_area / frame_area

            # nếu chiếm nhiều diện tích => coi là gần
            if ratio >= args.near_ratio:
                label = f"OBSTACLE NEAR ({int(ratio*100)}%)"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)  # đỏ
            else:
                label = f"moving/far ({int(ratio*100)}%)"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)  # vàng

        # FPS
        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        cv2.putText(frame, f"{label} | FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Obstacle Detect", frame)
        if args.show_mask:
            cv2.imshow("Mask", fg)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
