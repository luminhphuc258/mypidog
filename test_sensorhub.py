import serial

# Tùy máy có thể là ttyUSB0 hoặc ttyACM0
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print("Dữ liệu nhận từ ESP32:", data)
