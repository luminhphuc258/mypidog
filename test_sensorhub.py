import serial
import csv
import time

# Cấu hình cổng UART kết nối ESP32 → Raspberry Pi
SERIAL_PORT = "/dev/ttyUSB0"   # ⚠️ Kiểm tra lại bằng `ls /dev/ttyUSB*`
BAUD_RATE = 115200             # ⚠️ Phải khớp với tốc độ bên Arduino

# File log CSV sẽ lưu
LOG_FILE = "sensor_log.csv"

# Kết nối UART
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Đợi ESP32 khởi động

print("Đang ghi dữ liệu từ ESP32... Nhấn Ctrl+C để dừng")

# Ghi tiêu đề cột
with open(LOG_FILE, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp_ms", "tempC", "humidity", "ultrasonic_cm"])

try:
    while True:
        raw = ser.readline()
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                print("UART >>", line)
                parts = line.split(",")

                if len(parts) == 4:
                    with open(LOG_FILE, mode="a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow(parts)
        except Exception as e:
            print("Lỗi xử lý chuỗi:", e)

except KeyboardInterrupt:
    print("\nĐã dừng ghi dữ liệu.")

finally:
    ser.close()
