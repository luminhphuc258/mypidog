#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Matthew Robot - Raspberry Pi Auto Listening (WAV Mode)
Dành riêng cho API NodeJS mới: /pi_upload_audio
"""

import os
import time
import wave
import struct
import tempfile
import subprocess
import requests

# ==============================
# CONFIG
# ==============================

MIC_DEVICE = "plughw:4,0"       # USB Mic
SPEAKER_DEVICE = "plughw:3,0"   # PiDog speaker

SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = "S16_LE"

DETECT_CHUNK_SEC = 1.0     # tăng lên để chống noise từ quạt
RECORD_SEC = 4.0           # thời gian ghi câu nói

THRESHOLD = 2000           # NGƯỠNG KÍCH HOẠT (tăng lên nếu noise mạnh)

NODEJS_BASE = "https://embeddedprogramming-healtheworldserver.up.railway.app"
PI_UPLOAD_URL = NODEJS_BASE + "/pi_upload_audio"


# ==============================
# GHI ÂM WAV
# ==============================

def record_wav(filename: str, seconds: float):
    cmd = [
        "arecord",
        "-D", MIC_DEVICE,
        "-f", FORMAT,
        "-r", str(SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-d", str(seconds),
        filename,
    ]
    subprocess.run(cmd, check=False)


# ==============================
# ĐO BIÊN ĐỘ PCM
# ==============================

def get_max_amplitude(path: str) -> int:
    try:
        with wave.open(path, "rb") as wf:
            frames = wf.getnframes()
            raw = wf.readframes(frames)
            if not raw:
                return 0

            samples = struct.unpack("<" + "h" * (len(raw) // 2), raw)
            peak = max(abs(s) for s in samples)
            return peak
    except:
        return 0


# ==============================
# UPLOAD WAV → SERVER
# ==============================

def upload_to_server(wav_path: str):
    print(f"[UPLOAD] Sending {wav_path} -> /pi_upload_audio")
    try:
        with open(wav_path, "rb") as f:
            files = {"audio": ("voice.wav", f, "audio/wav")}
            resp = requests.post(PI_UPLOAD_URL, files=files, timeout=120)

        resp.raise_for_status()
        data = resp.json()
        print("[SERVER] Response:", data)
        return data
    except Exception as e:
        print("[UPLOAD ERROR]", e)
        return None


# ==============================
# DOWNLOAD AUDIO TRẢ LỜI
# ==============================

def download_audio(url: str):
    try:
        r = requests.get(url, stream=True, timeout=120)
        r.raise_for_status()

        fd, tmp_path = tempfile.mkstemp(suffix=".mp3")
        os.close(fd)
        with open(tmp_path, "wb") as f:
            for chunk in r.iter_content(8192):
                if chunk:
                    f.write(chunk)
        return tmp_path
    except:
        return None


# ==============================
# PHÁT AUDIO
# ==============================

def play_audio(filepath: str):
    print(f"[PLAY] {filepath}")
    cmd = ["aplay", "-D", SPEAKER_DEVICE, filepath]
    subprocess.run(cmd, check=False)


# ==============================
# MAIN LOOP
# ==============================

def main():
    print("=== Matthew Robot — Auto Listening (Raspberry Pi Mode) ===")
    print("Mic:", MIC_DEVICE)
    print("Speaker:", SPEAKER_DEVICE)
    print("Threshold:", THRESHOLD)
    print("Nhấn Ctrl+C để dừng\n")

    while True:
        try:
            # 1) GHI ĐOẠN NGẮN ĐỂ DÒ NGƯỠNG
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                detect_path = tmp.name

            record_wav(detect_path, DETECT_CHUNK_SEC)
            level = get_max_amplitude(detect_path)
            os.unlink(detect_path)

            print(f"[LISTEN] Level = {level}  (threshold={THRESHOLD})")

            if level < THRESHOLD:
                continue

            print("=== TRIGGER! Bắt đầu ghi câu nói ===")
            fd, voice_path = tempfile.mkstemp(suffix=".wav")
            os.close(fd)

            record_wav(voice_path, RECORD_SEC)

            # 2) UPLOAD WAV
            data = upload_to_server(voice_path)

            try:
                os.unlink(voice_path)
            except:
                pass

            if not data:
                print("[MAIN] Server không phản hồi. Tiếp tục nghe.")
                continue

            text = data.get("text", "")
            label = data.get("label", "")
            audio_url = data.get("audio_url")

            print("[TRANSCRIPT]:", text)
            print("[LABEL]:", label)

            if not audio_url:
                print("[MAIN] Không có audio trả về.")
                continue

            # 3) TẢI FILE ÂM THANH TRẢ LỜI
            reply_path = download_audio(audio_url)
            if reply_path:
                play_audio(reply_path)
                os.unlink(reply_path)

            print("\n--- Vòng lặp xong, quay lại listening ---\n")

        except KeyboardInterrupt:
            print("\n[MAIN] Stop by user.")
            break
        except Exception as e:
            print("[MAIN ERROR]", e)
            time.sleep(1)


if __name__ == "__main__":
    main()
