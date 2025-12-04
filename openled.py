#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RGB LED only for PiDog - avoid importing/initializing Pidog() to prevent motor reset.

Usage examples:
  sudo python3 rgb_led_only_no_pidog.py --color blue --style breath
  sudo python3 rgb_led_only_no_pidog.py --color "255,0,0" --style solid
  sudo python3 rgb_led_only_no_pidog.py --off
"""

import time
import argparse

def parse_color(s: str):
    s = s.strip()
    if "," in s:
        parts = [p.strip() for p in s.split(",")]
        if len(parts) != 3:
            raise ValueError("Color must be name (e.g. blue) or 'r,g,b' (e.g. 0,0,255)")
        r, g, b = [int(x) for x in parts]
        return [max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))]
    return s  # color name like "blue", "pink", ...

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--style", default="breath", help="solid / breath / blink / listen / ... (depends on RGBStrip.STYLES)")
    ap.add_argument("--color", default="blue", help="color name or r,g,b (e.g. 0,0,255)")
    ap.add_argument("--bps", type=float, default=1.2, help="beats per second (effect speed)")
    ap.add_argument("--brightness", type=float, default=0.8, help="0..1")
    ap.add_argument("--seconds", type=float, default=10.0, help="run duration")
    ap.add_argument("--off", action="store_true", help="turn off and exit")
    args = ap.parse_args()

    # IMPORTANT: no Pidog() here
    from pidog.rgb_strip import RGBStrip

    strip = None
    try:
        strip = RGBStrip()

        if args.off:
            # try turn off in a safe way
            try:
                strip.set_mode(style="solid", color=[0, 0, 0], bps=1, brightness=0)
            except Exception:
                pass
            try:
                strip.close()
            except Exception:
                pass
            return

        color = parse_color(args.color)

        # set mode
        strip.set_mode(style=args.style, color=color, bps=args.bps, brightness=args.brightness)

        # Some implementations need periodic "show()" to animate.
        # We'll call show() in a loop to be safe (won't touch servos).
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            try:
                strip.show()
            except Exception:
                # if show() not required or not available, ignore
                pass
            time.sleep(0.02)

    finally:
        if strip is not None:
            try:
                strip.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()