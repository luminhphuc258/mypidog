#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Matthew Robot – Realtime Voice Detection + Upload + Playback
Không dùng -d FLOAT → không bị lỗi. Nghe realtime bằng PIPE.
"""

import os
import time
import wave
import struct
import tempfile
import subprocess
import requests
import threading

# ========== CONFIG ==========
MIC_DEVICE = "plughw:4,0"
SPEAKER_DEVICE = "plughw:3,0"

SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = "S16_LE"

FRAME_SIZE = 2048            # Số bytes đọc mỗi lần (~0.064s)
AMP_THRESHOLD = 1500         # Ngưỡng kích hoạt
MAX_SILENCE_AFTER_TALK = 20  # Sau bao nhiêu frame yên thì ngừng ghi
NODEJS_URL = "https://embeddedprogramming-healtheworldserver.up.railway.app/upload_audio"
# ============================


def start_arecord_stream():
    """Mở 1 process arecord chạy liên tục và trả về PIPE."""
    cmd = [
        "arecord",
        "-D", MIC_DEVICE,
        "-f", FORMAT,
        "-r", str(SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-t", "raw",          # Lấy raw PCM
        "-q"
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE)


def pcm_max_amp(raw_bytes: bytes) -> int:
    """Tính biên độ max của buffer raw PCM."""
    if not raw_bytes:
        return 0
    samples = struct.unpack("<" + "h" * (len(raw_bytes) // 2), raw_bytes)
    return max(abs(s) for s in samples)


def save_wav(filename, pcm_data: bytes):
    """Lưu raw PCM thành file WAV."""
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm_data)


def upload_audio(filepath):
    """Upload WAV lên NodeJS."""
    try:
        with open(filepath, "rb") as f:
            resp = requests.post(NODEJS_URL, files={"audio": f}, timeout=60)
        resp.raise_for_status()
        return resp.json()
    except Exception as e:
        print("[UPLOAD] Lỗi:", e)
        return None


def download_audio(url):
    """Tải file WAV phản hồi từ server."""
    try:
        r = requests.get(url, stream=True, timeout=60)
        r.raise_for_status()
        fd, path = tempfile.mkstemp(suffix=".wav", prefix="reply_")
        os.close(fd)
        with open(path, "wb") as f:
            for chunk in r.iter_content(8192):
                f.write(chunk)
        return path
    except Exception as e:
        print("[DOWNLOAD] Lỗi:", e)
        return None


def play_audio(filepath):
    """Phát WAV qua loa PiDog."""
    cmd = ["aplay", "-D", SPEAKER_DEVICE, "-q", filepath]
    subprocess.run(cmd, check=False)


def main():
    print("=== Matthew Robot – Real-time Voice Listening ===")
    print("Mic:", MIC_DEVICE, " Loa:", SPEAKER_DEVICE)
    print("Nhấn Ctrl+C để thoát.\n")

    arec = start_arecord_stream()
    recording = False
    pcm_buffer = b""
    silence_count = 0

    try:
        while True:
            raw = arec.stdout.read(FRAME_SIZE)
            if not raw:
                continue

            amp = pcm_max_amp(raw)
            print(f"[LISTEN] level={amp}")

            if not recording:
                # Chưa ghi, xem có trigger không
                if amp > AMP_THRESHOLD:
                    print("\n=== TRIGGER START – bắt đầu ghi ===")
                    recording = True
                    pcm_buffer = raw
                    silence_count = 0
                continue

            # Đang ghi
            pcm_buffer += raw

            if amp < AMP_THRESHOLD:
                silence_count += 1
            else:
                silence_count = 0

            # Nếu im lặng quá lâu thì dừng ghi
            if silence_count > MAX_SILENCE_AFTER_TALK:
                print("=== TRIGGER END – ghi xong ===")

                # Lưu file WAV
                fd, wav_path = tempfile.mkstemp(suffix=".wav", prefix="utter_")
                os.close(fd)
                save_wav(wav_path, pcm_buffer)

                # Upload
                print("[UPLOAD] Uploading...")
                resp = upload_audio(wav_path)
                os.unlink(wav_path)

                if resp and resp.get("audio_url"):
                    print("[SERVER] Transcript:", resp.get("transcript"))
                    print("[SERVER] Label:", resp.get("label"))

                    # Tải audio trả lời
                    reply = download_audio(resp["audio_url"])
                    if reply:
                        play_audio(reply)
                        os.unlink(reply)

                recording = False
                pcm_buffer = b""
                print("=== Ready, quay lại listening ===\n")

    except KeyboardInterrupt:
        print("\n[MAIN] Thoát.")
    finally:
        arec.kill()


if __name__ == "__main__":
    main()
