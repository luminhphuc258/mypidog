from pidog import Pidog
import time

dog = Pidog()

print("=== TEST IMU (Gia tốc + Góc nghiêng) ===")
print("Nhấn Ctrl+C để dừng...\n")

while True:
    # Đọc gia tốc
    ax, ay, az = dog.imu.read_accel()

    # Đọc gyro
    gx, gy, gz = dog.imu.read_gyro()

    # Đọc góc nghiêng do IMU tính sẵn
    roll, pitch, yaw = dog.imu.get_angle()

    print(f"ACC: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f} | "
          f"GYRO: gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f} | "
          f"ANGLE: roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°")

    time.sleep(0.2)
