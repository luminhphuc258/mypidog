#!/usr/bin/env python3
import subprocess
import time

OUTPUT = "default"   # đúng theo xrandr của bạn

def rotate_180():
    subprocess.run(
        ["xrandr", "--output", OUTPUT, "--rotate", "inverted"],
        check=False
    )

if __name__ == "__main__":
    # đợi desktop load xong
    time.sleep(2)
    rotate_180()
    print("Screen rotated 180° (inverted)")
