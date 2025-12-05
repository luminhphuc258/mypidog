#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time

# File wav có sẵn trong hệ thống
BARK_WAV = "/usr/share/sounds/alsa/Front_Center.wav"

# Thử lần lượt 3 device: default, plughw:3,0, hw:3,0
DEVICES = [
    "default",
    "plughw:3,0",
    "hw:3,0",
]

def play_with_device(device: str):
    print(f"\n=== Thử phát với ALSA device: {device} ===")
    cmd = ["aplay", "-D", device, "-q", BARK_WAV]
    try:
        subprocess.run(cmd, check=False)
        print(f"→ Lệnh aplay với '{device}' đã chạy xong.")
    except FileNotFoundError:
        print("❌ Không tìm thấy 'aplay'. Cài: sudo apt install alsa-utils")

def main():
    print("Test đơn giản: phát tiếng 'bark' qua loa (KHÔNG dùng Pidog / servo).")
    print(f"File đang dùng: {BARK_WAV}")

    for dev in DEVICES:
        play_with_device(dev)
        time.sleep(1)

    print("\nKết thúc test.")

if __name__ == "__main__":
    main()
