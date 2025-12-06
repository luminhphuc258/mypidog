#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# check mic ne: fuser -v /dev/snd/*

"""
Matthew Robot – Auto Listening on Raspberry Pi
- Ghi âm theo ngưỡng
- Upload audio lên NodeJS
- Nhận lại file âm thanh và phát qua loa PiDog
"""

import os
import time
import wave
import struct
import tempfile
import subprocess
import requests

# ================== CONFIG ==================

MIC_DEVICE = "plughw:4,0"        # USB microphone
SPEAKER_DEVICE = "plughw:3,0"    # PiDog speaker

SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = "S16_LE"

DETECT_CHUNK_SEC = 1          # PHẢI LÀ SỐ NGUYÊN
RECORD_SEC = 4                # PHẢI LÀ SỐ NGUYÊN

THRESHOLD = 2500              # tăng lên để tránh tiếng quạt

NODEJS_BASE = "https://embeddedprogramming-healtheworldserver.up.railway.app"
NODEJS_UPLOAD_URL = NODEJS_BASE + "/upload_audio"


# ================== HELPERS ==================

def record_wav(filename, seconds):
    """
    Ghi 1 đoạn WAV bằng arecord.
    IMPORTANT: arecord KHÔNG CHO PHÉP số float → phải int().
    """
    cmd = [
        "arecord",
        "-D", MIC_DEVICE,
        "-f", FORMAT,
        "-r", str(SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-d", str(int(seconds)),   # FIX LỖI "invalid duration argument"
        "-q",
        filename,
    ]

    subprocess.run(cmd, check=False)


def get_max_amplitude(filename):
    """Trả về biên độ max của WAV (0..32767)."""
    try:
        with wave.open(filename, "rb") as wf:
            raw = wf.readframes(wf.getnframes())
            if not raw:
                return 0

        # unpack 16-bit PCM
        samples = struct.unpack("<" + "h" * (len(raw) // 2), raw)
        return max(abs(s) for s in samples)
    except Exception:
        return 0


def upload_to_node(filepath):
    """Upload file WAV lên NodeJS."""
    print(f"[UPLOAD] Uploading...")

    try:
        with open(filepath, "rb") as f:
            files = {"audio": ("voice.wav", f, "audio/wav")}
            resp = requests.post(NODEJS_UPLOAD_URL, files=files, timeout=60)

        resp.raise_for_status()
        data = resp.json()
        return data
    except Exception as e:
        print("[UPLOAD ERROR]", e)
        return None


def download_audio(url):
    """Download audio trả về của NodeJS."""
    print("[DOWNLOAD] From:", url)

    try:
        r = requests.get(url, stream=True, timeout=60)
        r.raise_for_status()

        fd, path = tempfile.mkstemp(suffix=".mp3", prefix="reply_")
        os.close(fd)

        with open(path, "wb") as f:
            for chunk in r.iter_content(8192):
                if chunk:
                    f.write(chunk)

        print("[DOWNLOAD] Saved:", path)
        return path
    except Exception as e:
        print("[DOWNLOAD ERROR]", e)
        return None


def play_audio(filepath):
    print(f"[PLAY] Đang choi nhac {filepath}")
    
    # Nếu file là mp3 → convert sang wav
    if filepath.endswith(".mp3"):
        filepath = convert_mp3_to_wav(filepath)

    cmd = ["aplay", "-D", SPEAKER_DEVICE, "-q", filepath]
    subprocess.run(cmd, check=False)
    print("[PLAY] Done.")


def convert_mp3_to_wav(mp3_path):
    wav_path = mp3_path.replace(".mp3", ".wav")
    cmd = [
        "ffmpeg",
        "-y",
        "-i", mp3_path,
        "-ac", "1",
        "-ar", "16000",
        wav_path
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return wav_path
import os
import shutil
import subprocess
import wave

CONFIG_PATHS = ["/boot/firmware/config.txt", "/boot/config.txt"]

def _read_boot_config():
    for p in CONFIG_PATHS:
        if os.path.exists(p):
            try:
                return open(p, "r", errors="ignore").read()
            except:
                pass
    return ""

def detect_spk_en_pin():
    """
    SunFounder:
      - googlevoicehat-soundcard (có mic onboard)  -> SPK_EN = GPIO12
      - hifiberry-dac (không mic onboard)          -> SPK_EN = GPIO20
    """
    txt = _read_boot_config()
    # lấy dòng dtoverlay không bị comment
    lines = [ln.split("#", 1)[0].strip() for ln in txt.splitlines()]
    overlays = [ln for ln in lines if ln.startswith("dtoverlay=")]

    if any("googlevoicehat-soundcard" in ln for ln in overlays):
        return 12
    if any("hifiberry-dac" in ln for ln in overlays):
        return 20

    # fallback: nhìn aplay -l có card googlevoicehat không
    try:
        out = subprocess.run(["aplay", "-l"], capture_output=True, text=True).stdout.lower()
        if "googlevoi" in out or "googlevoicehat" in out:
            return 12
    except:
        pass

    # fallback cuối: đa số robothat không mic -> 20
    return 20

def set_gpio_high(pin):
    """
    Bật SPK_EN lên HIGH. Cần quyền root (chạy script bằng sudo).
    """
    if shutil.which("pinctrl"):
        subprocess.run(["pinctrl", "set", str(pin), "op", "dh"], check=False)
        return True

    if shutil.which("raspi-gpio"):
        subprocess.run(["raspi-gpio", "set", str(pin), "op", "dh"], check=False)
        return True

    return False

def prime_speaker_aplay(device):
    """
    Phát 0.1s silence để 'prime' đường audio (giống play -n trim 0 0.5).
    """
    silence = "/tmp/robothat_silence.wav"
    if not os.path.exists(silence):
        with wave.open(silence, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)     # 16-bit
            wf.setframerate(16000)
            wf.writeframes(b"\x00\x00" * (16000 // 10))  # 0.1s
    subprocess.run(["aplay", "-D", device, "-q", silence], check=False)

def unlock_robothat_speaker(speaker_device):
    pin = detect_spk_en_pin()
    ok = set_gpio_high(pin)

    if not ok:
        print("[WARN] Không tìm thấy pinctrl/raspi-gpio hoặc không đủ quyền. Hãy chạy bằng sudo.")
        return False

    # prime để chắc chắn “mở khóa”
    prime_speaker_aplay(speaker_device)

    # (tuỳ chọn) set volume max nếu control tồn tại
    subprocess.run(["amixer", "sset", "robot-hat speaker", "100%"], check=False)

    print(f"[OK] Speaker unlocked (SPK_EN GPIO{pin})")
    return True



# ================== MAIN LOOP ==================

def main_loop():
    unlock_robothat_speaker(SPEAKER_DEVICE)
    print("\n=== Matthew Robot — Auto Listening ===")
    print(f"Mic: {MIC_DEVICE}")
    print(f"Speaker: {SPEAKER_DEVICE}")
    print(f"Threshold: {THRESHOLD}")
    print("Nhấn Ctrl+C để dừng.\n")

    while True:
        try:
            # --- Step 1: listen small chunk ---
            tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
            detect_file = tmp.name
            tmp.close()

            record_wav(detect_file, DETECT_CHUNK_SEC)

            level = get_max_amplitude(detect_file)
            os.unlink(detect_file)

            print(f"[LISTEN] Level = {level} (threshold={THRESHOLD})")

            if level < THRESHOLD:
                continue

            # --- Step 2: record full sentence ---
            print("\n=== TRIGGER! Âm thanh vượt ngưỡng ===")
            print("→ Đang ghi câu nói...\n")

            fd, record_file = tempfile.mkstemp(suffix=".wav", prefix="voice_")
            os.close(fd)

            record_wav(record_file, RECORD_SEC)
            print("=== TRIGGER END — ghi xong ===")

            # --- Step 3: upload to server ---
            resp = upload_to_node(record_file)

            try:
                os.unlink(record_file)
            except:
                pass

            if not resp:
                print("[ERROR] Server không trả về JSON\n")
                continue

            transcript = resp.get("transcript", "")
            label = resp.get("label", "unknown")
            audio_url = resp.get("audio_url", None)

            print("[SERVER] Transcript:", transcript)
            print("[SERVER] Label:", label)

            if not audio_url:
                print("[SERVER] Không có audio trả về.\n")
                continue

            # --- Step 4: download TTS and play ---
            reply_file = download_audio(audio_url)

            if reply_file:
                play_audio(reply_file)
                os.unlink(reply_file)

            print("\n=== HOÀN TẤT 1 VÒNG — Quay lại listening... ===\n")
            time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[EXIT] Đã thoát.")
            break

        except Exception as e:
            print("[MAIN ERROR]", e)
            time.sleep(1)


if __name__ == "__main__":
    main_loop()
