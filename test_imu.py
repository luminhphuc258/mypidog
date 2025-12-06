from pidog import Pidog
import time

dog = Pidog()

print("=== TEST IMU SH3001 ===")
print("Nhấn Ctrl+C để dừng...\n")

while True:
    # Đọc gia tốc
    ax, ay, az = dog.imu.get_acc()

    # Đọc gyro
    gx, gy, gz = dog.imu.get_gyro()

    # Đọc góc roll, pitch, yaw
    roll, pitch, yaw = dog.imu.get_rpy()

    print(f"ACC: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f} | "
          f"GYRO: gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f} | "
          f"RPY: roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°")

    time.sleep(0.2)
