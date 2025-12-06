#!/usr/bin/env python3
import subprocess
import re

def get_playback_devices():
    """Lấy danh sách device từ aplay -L (PCM name)"""
    out = subprocess.check_output(["aplay", "-L"], text=True)
    devices = []
    for line in out.splitlines():
        if line.startswith("plughw:") or line.startswith("hw:"):
            devices.append(line.strip())
    return devices

def play_test(device):
    """Phát tiếng beep thử lên thiết bị"""
    print(f"\n=== TEST {device} ===")
    try:
        subprocess.run(
            ["speaker-test", "-D", device, "-t", "sine", "-f", "440", "-l", "1"],
            timeout=4
        )
        print(f"[OK] {device} phát được!")
    except subprocess.TimeoutExpired:
        print(f"[OK] {device} phát được (timeout sau 1 loop).")
    except Exception as e:
        print(f"[FAIL] {device} lỗi: {e}")

def main():
    print("🔍 Đang dò tất cả thiết bị âm thanh...\n")
    devices = get_playback_devices()

    if not devices:
        print("❌ Không tìm thấy thiết bị nào!")
        return

    print(f"Tìm thấy {len(devices)} device:\n")
    for d in devices:
        print(" -", d)

    print("\n🔊 Bắt đầu test từng thiết bị...\n")
    for d in devices:
        play_test(d)

    print("\n✅ Test xong. Thiết bị nào [OK] là loa đang hoạt động!")
    print("Nếu KHÔNG cái nào phát tiếng → lỗi phần cứng hoặc I2S chưa được enable.")

if __name__ == "__main__":
    main()
