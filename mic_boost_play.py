#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ghi âm vài giây từ micro PiDog (Google VoiceHAT),
sau đó tự động BOOST âm lượng rồi phát lại qua loa.

- Không dùng Pidog() nên sẽ không reset servo.
- Dùng card: plughw:3,0 (snd_rpi_googlevoicehat_soundcar)
"""

import subprocess
from pathlib import Path
import wave
import numpy as np
import time

# ==== CONFIG ====
CARD_DEVICE = "plughw:3,0"      # card 3,0 cho voiceHAT
REC_SECONDS = 4                 # thời gian ghi âm
RAW_WAV = Path("raw_mic.wav")   # file gốc
BOOSTED_WAV = Path("boosted_mic.wav")  # file sau khi boost

# mong muốn peak ~ -3 dB (0 dB là max, -3 dB an toàn không vỡ)
TARGET_PEAK_DB = -3.0  


def record_from_mic():
    """Ghi âm REC_SECONDS giây từ micro vào RAW_WAV."""
    if RAW_WAV.exists():
        RAW_WAV.unlink()

    cmd = [
        "arecord",
        "-D", CARD_DEVICE,
        "-f", "S16_LE",          # 16-bit
        "-r", "16000",           # 16 kHz
        "-c", "1",               # mono
        "-d", str(REC_SECONDS),  # thời gian
        "-q",                    # quiet
        str(RAW_WAV),
    ]
    print(f"[REC] Ghi âm {REC_SECONDS}s từ {CARD_DEVICE} -> {RAW_WAV}")
    subprocess.run(cmd, check=False)
    print("[REC] xong.")


def boost_wav(input_path: Path, output_path: Path,
              target_peak_db: float = TARGET_PEAK_DB) -> float:
    """
    Đọc file WAV 16-bit, scale âm lượng sao cho peak ~ target_peak_db.
    Trả về gain_db đã áp dụng.
    """
    if not input_path.exists():
        print(f"[BOOST] Không tìm thấy file: {input_path}")
        return 0.0

    with wave.open(str(input_path), "rb") as wf:
        params = wf.getparams()
        n_frames = wf.getnframes()
        audio_bytes = wf.readframes(n_frames)

    # 16-bit signed -> int16
    samples = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32)
    if samples.size == 0:
        print("[BOOST] File trống, không có dữ liệu.")
        return 0.0

    # Tìm peak hiện tại (0..32767)
    peak = np.max(np.abs(samples))
    if peak <= 0:
        print("[BOOST] Peak = 0 (toàn im lặng), không boost được.")
        gain_db = 0.0
    else:
        # peak hiện tại tính theo dBFS
        peak_db = 20.0 * np.log10(peak / 32767.0)
        gain_db = target_peak_db - peak_db
        # giới hạn tối đa, tránh noise quá to
        gain_db = min(gain_db, 30.0)     # max +30 dB
        gain_db = max(gain_db, 0.0)      # không giảm, chỉ tăng
    gain = 10.0 ** (gain_db / 20.0)

    print(f"[BOOST] peak cũ: ~{peak:.0f}, gain_db áp dụng: +{gain_db:.1f} dB")

    boosted = samples * gain
    boosted = np.clip(boosted, -32768, 32767).astype(np.int16)

    with wave.open(str(output_path), "wb") as wf:
        wf.setparams(params)
        wf.writeframes(boosted.tobytes())

    print(f"[BOOST] Đã lưu file boost -> {output_path}")
    return gain_db


def play_to_speaker(path: Path):
    """Phát file WAV ra loa voiceHAT."""
    if not path.exists():
        print(f"[PLAY] Không tìm thấy file: {path}")
        return

    cmd = [
        "aplay",
        "-D", CARD_DEVICE,
        "-q",
        str(path),
    ]
    print(f"[PLAY] Phát file {path} qua {CARD_DEVICE} ...")
    subprocess.run(cmd, check=False)
    print("[PLAY] xong.")


def main():
    print("=== Demo ghi âm + BOOST + phát lại ===")
    print("Nói vào micro trong khi ghi...")

    # 1) Ghi âm
    record_from_mic()

    # 2) Boost âm lượng
    gain_db = boost_wav(RAW_WAV, BOOSTED_WAV, TARGET_PEAK_DB)

    # 3) Phát lại
    if gain_db > 0:
        print(f"[INFO] Đã boost thêm khoảng +{gain_db:.1f} dB.")
    else:
        print("[INFO] Không boost (file có thể im lặng hoặc đã to sẵn).")

    time.sleep(0.5)
    play_to_speaker(BOOSTED_WAV)

    print("=== Kết thúc demo ===")


if __name__ == "__main__":
    main()
