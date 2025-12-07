#!/usr/bin/env python3
import serial
import time

PORT = "/dev/serial0"   # đổi thành "/dev/ttyAMA0" nếu cần
BAUD = 115200           # TF-Luna default
THRESH_CM = 60          # dưới 60cm coi là có vật cản
MIN_STRENGTH = 50       # lọc nhiễu, tùy chỉnh
STABLE_HITS = 3         # cần 3 lần liên tiếp dưới ngưỡng mới báo obstacle

def read_tfluna_frame(ser):
    """
    TF-Luna frame 9 bytes:
    [0]=0x59 [1]=0x59 [2]=Dist_L [3]=Dist_H [4]=Strength_L [5]=Strength_H
    [6]=Temp_L [7]=Temp_H [8]=Checksum
    dist: cm
    """
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0x59:
            continue

        b2 = ser.read(1)
        if not b2 or b2[0] != 0x59:
            continue

        payload = ser.read(7)
        if len(payload) != 7:
            return None

        frame = bytes([0x59, 0x59]) + payload

        # checksum = sum(first 8 bytes) & 0xFF
        chk = sum(frame[0:8]) & 0xFF
        if chk != frame[8]:
            continue

        dist = frame[2] | (frame[3] << 8)          # cm
        strength = frame[4] | (frame[5] << 8)
        temp_raw = frame[6] | (frame[7] << 8)
        temp_c = temp_raw / 8.0 - 256.0
        return dist, strength, temp_c

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(0.2)

    print(f"TF-Luna UART test on {PORT} @ {BAUD}")
    stable = 0

    while True:
        data = read_tfluna_frame(ser)
        if data is None:
            print("No valid frame (check wiring/baud/serial port)...")
            stable = 0
            continue

        dist, strength, temp_c = data

        # lọc giá trị rác
        valid = (dist > 0 and dist < 1200 and strength >= MIN_STRENGTH)
        obstacle_now = valid and (dist <= THRESH_CM)

        if obstacle_now:
            stable += 1
        else:
            stable = 0

        status = "OBSTACLE!" if stable >= STABLE_HITS else "clear"
        print(f"dist={dist:4d} cm | strength={strength:5d} | temp={temp_c:5.1f}C | {status}")

        time.sleep(0.05)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
