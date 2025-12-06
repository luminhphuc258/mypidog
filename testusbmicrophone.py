#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
from pathlib import Path
import wave
import numpy as np
import time

from robot_hat import Sound

# Kích hoạt hệ thống audio
snd = Sound()

print("Audio device initialized!")


# ==== DEVICE CỤ THỂ TỪ OUTPUT CỦA BẠN ====
USB_MIC_DEVICE = "plughw:4,0"     # USB PnP Microphone
SPEAKER_DEVICE = "plughw:3,0"     # Loa voiceHAT trên PiDog

# ==== FILE OUTPUT ====
RAW_WAV = Path("usb_raw.wav")
BOOSTED_WAV = Path("usb_boosted.wav")

# ==== BOOST CONFIG ====
REC_SECONDS = 8
TARGET_PEAK_DB = -6.0      # normalize thấp hơn một chút
EXTRA_GAIN_DB  = 6.0       # thêm +6 dB thôi
MAX_TOTAL_GAIN_DB = 35.0 


def record_usb_mic():
    """Ghi âm từ USB mic."""
    if RAW_WAV.exists():
        RAW_WAV.unlink()

    print(f"[REC] Ghi âm {REC_SECONDS}s từ USB mic ({USB_MIC_DEVICE}) ...")

    cmd = [
        "arecord",
        "-D", USB_MIC_DEVICE,
        "-f", "S16_LE",
        "-r", "16000",
        "-c", "1",
        "-d", str(REC_SECONDS),
        "-q",
        str(RAW_WAV)
    ]

    subprocess.run(cmd)
    print("[REC] Đã ghi xong.")


def boost_audio(input_wav: Path, output_wav: Path):
    """Normalize + boost âm lượng."""
    print("[BOOST] Đang xử lý âm lượng...")

    with wave.open(str(input_wav), "rb") as wf:
        params = wf.getparams()
        frames = wf.readframes(wf.getnframes())

    samples = np.frombuffer(frames, dtype=np.int16).astype(np.float32)

    if samples.size == 0:
        print("[BOOST] File trống.")
        return

    peak = np.max(np.abs(samples))
    peak_db = 20 * np.log10(peak / 32767.0)

    gain_db = (TARGET_PEAK_DB - peak_db) + EXTRA_GAIN_DB
    gain_db = min(gain_db, MAX_TOTAL_GAIN_DB)

    gain = 10 ** (gain_db / 20)

    print(f"[BOOST] Peak cũ: {peak_db:.1f} dB → áp gain: +{gain_db:.1f} dB")

    boosted = np.clip(samples * gain, -32768, 32767).astype(np.int16)

    with wave.open(str(output_wav), "wb") as wf:
        wf.setparams(params)
        wf.writeframes(boosted.tobytes())

    print(f"[BOOST] Đã lưu file: {output_wav}")


def play_on_speaker(path: Path):
    """Phát wav ra loa PiDog."""
    print("[PLAY] Phát ra loa robot...")

    cmd = ["aplay", "-D", SPEAKER_DEVICE, "-q", str(path)]
    subprocess.run(cmd)

    print("[PLAY] Xong.")


def main():
    print("=== TEST USB MICROPHONE (card 4, device 0) ===\n")

    record_usb_mic()
    boost_audio(RAW_WAV, BOOSTED_WAV)

    time.sleep(0.5)
    play_on_speaker(BOOSTED_WAV)

    print("\n=== Hoàn tất ===")


if __name__ == "__main__":
    main()
