#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto active listening trên Raspberry Pi:

- Dùng USB microphone (ALSA device MIC_DEVICE) để liên tục "nghe".
- Mỗi vòng lặp ghi 1 đoạn ngắn (DETECT_CHUNK_SEC), đo biên độ âm thanh.
- Nếu biên độ > THRESHOLD  => ghi 1 câu nói (RECORD_SEC) vào file WAV.
- Gửi file đó lên NodeJS /upload_audio (field name: "audio").
- Nhận JSON có audio_url, tải file audio về rồi phát qua loa PiDog
  (ALSA device SPEAKER_DEVICE) bằng aplay.
- Trong lúc upload + phát, script dừng ghi; phát xong mới quay lại vòng nghe tiếp.
"""

import os
import time
import wave
import struct
import tempfile
import subprocess
import requests

# ================== CONFIG ==================

# --- Thiết bị ALSA ---
# USB mic (arecord -l để kiểm tra lại)
MIC_DEVICE = "plughw:4,0"

# Loa PiDog (aplay -l để kiểm tra lại)
SPEAKER_DEVICE = "plughw:3,0"

# --- Tham số ghi âm ---
SAMPLE_RATE = 16000    # Hz
CHANNELS = 1
FORMAT = "S16_LE"      # 16-bit signed little endian

DETECT_CHUNK_SEC = 0.7   # thời gian mỗi "chunk" để dò ngưỡng
RECORD_SEC = 4.0         # thời gian ghi khi đã trigger

# Ngưỡng biên độ PCM (0..32767). Cần thử thực tế để chỉnh.
#  - Nếu kích hoạt hoài thì tăng THRESHOLD.
#  - Nếu gọi to mà vẫn không kích hoạt thì giảm THRESHOLD.
THRESHOLD = 1500

# URL NodeJS (giống code Flask của bạn)
NODEJS_BASE = "https://embeddedprogramming-healtheworldserver.up.railway.app"
NODEJS_UPLOAD_URL = NODEJS_BASE + "/upload_audio"

# ===========================================


def record_wav(filename: str, seconds: float):
    """
    Ghi audio từ USB mic vào file WAV bằng arecord.
    """
    cmd = [
        "arecord",
        "-D", MIC_DEVICE,
        "-f", FORMAT,
        "-r", str(SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-d", str(seconds),
        "-q",              # quiet
        filename,
    ]
    # check=False để nếu thỉnh thoảng có lỗi nhỏ thì không crash cả script
    subprocess.run(cmd, check=False)


def get_max_amplitude(filename: str) -> int:
    """
    Đọc file WAV, trả về biên độ |max| của mẫu PCM (0..32767).
    """
    try:
        with wave.open(filename, "rb") as wf:
            n_frames = wf.getnframes()
            raw = wf.readframes(n_frames)
            if not raw:
                return 0

            # 16-bit signed => 'h'
            samples = struct.unpack("<" + "h" * (len(raw) // 2), raw)
            max_amp = max(abs(s) for s in samples) if samples else 0
            return max_amp
    except Exception as e:
        print("[AMP] Lỗi đọc WAV:", e)
        return 0


def upload_to_node(filepath: str):
    """
    Gửi file WAV lên NodeJS bằng field 'audio'.
    Trả về dict JSON (hoặc None nếu lỗi).
    """
    print(f"[UPLOAD] Gửi {filepath} lên {NODEJS_UPLOAD_URL} ...")

    try:
        with open(filepath, "rb") as f:
            files = {
                "audio": ("voice.wav", f, "audio/wav")
            }
            resp = requests.post(NODEJS_UPLOAD_URL, files=files, timeout=120)
        resp.raise_for_status()
        data = resp.json()
        print("[UPLOAD] OK, server trả về JSON:", data)
        return data
    except Exception as e:
        print("[UPLOAD] Lỗi upload:", e)
        return None


def download_audio(url: str) -> str | None:
    """
    Tải file audio do NodeJS trả về về /tmp, trả lại đường dẫn file.
    """
    print("[DOWNLOAD] Tải audio từ:", url)
    try:
        r = requests.get(url, stream=True, timeout=120)
        r.raise_for_status()

        fd, path = tempfile.mkstemp(suffix=".wav", prefix="reply_")
        os.close(fd)  # sẽ chỉ ghi bằng file object khác
        with open(path, "wb") as f:
            for chunk in r.iter_content(8192):
                if chunk:
                    f.write(chunk)

        print(f"[DOWNLOAD] Lưu vào: {path}")
        return path
    except Exception as e:
        print("[DOWNLOAD] Lỗi tải audio:", e)
        return None


def play_audio(filepath: str):
    """
    Phát file WAV bằng aplay qua loa PiDog (SPEAKER_DEVICE).
    Trong lúc aplay chạy, function này block, nên không ghi âm.
    """
    print(f"[PLAY] Phát {filepath} qua {SPEAKER_DEVICE} ...")
    cmd = [
        "aplay",
        "-D", SPEAKER_DEVICE,
        "-q",
        filepath,
    ]
    try:
        subprocess.run(cmd, check=False)
        print("[PLAY] Phát xong.")
    except Exception as e:
        print("[PLAY] Lỗi phát audio:", e)


def main_loop():
    print("=== Matthew Robot - USB Mic Auto Listening ===")
    print(f"Mic: {MIC_DEVICE}, Loa: {SPEAKER_DEVICE}")
    print(f"Threshold: {THRESHOLD}")
    print("Nhấn Ctrl+C để dừng.\n")

    while True:
        try:
            # 1) Ghi 1 đoạn nhỏ để dò ngưỡng
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                detect_path = tmp.name

            record_wav(detect_path, DETECT_CHUNK_SEC)
            level = get_max_amplitude(detect_path)
            os.unlink(detect_path)  # xóa file nhỏ

            print(f"[LISTEN] Level={level} (threshold={THRESHOLD})")

            if level < THRESHOLD:
                # không đủ lớn, tiếp tục vòng lặp
                continue

            # 2) Đủ lớn => ghi câu nói đầy đủ
            print("[TRIGGER] Âm thanh vượt ngưỡng -> GHI CÂU NÓI...")
            fd, record_path = tempfile.mkstemp(suffix=".wav", prefix="utter_")
            os.close(fd)
            record_wav(record_path, RECORD_SEC)
            print("[TRIGGER] Ghi xong -> gửi lên server.")

            # 3) Upload lên NodeJS
            resp = upload_to_node(record_path)

            # Dọn file local (audio người dùng)
            try:
                os.unlink(record_path)
            except Exception:
                pass

            if not resp:
                print("[MAIN] Không có JSON trả về, quay lại listening...")
                time.sleep(0.5)
                continue

            audio_url = resp.get("audio_url")
            transcript = resp.get("transcript")
            label = resp.get("label")

            if transcript:
                print("[SERVER] Transcript:", transcript)
            if label:
                print("[SERVER] Label:", label)

            if not audio_url:
                print("[MAIN] Không có audio_url, quay lại listening...")
                time.sleep(0.5)
                continue

            # 4) Tải file audio trả về & phát
            reply_path = download_audio(audio_url)
            if reply_path:
                play_audio(reply_path)
                # xóa file trả lời nếu muốn
                try:
                    os.unlink(reply_path)
                except Exception:
                    pass

            # 5) Nghỉ 1 chút rồi quay lại nghe
            print("[MAIN] Hoàn tất 1 vòng, quay lại listening...\n")
            time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[MAIN] Nhận Ctrl+C, thoát.")
            break
        except Exception as e:
            print("[MAIN] Lỗi bất ngờ:", e)
            time.sleep(1.0)


if __name__ == "__main__":
    main_loop()
