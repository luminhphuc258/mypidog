#!/usr/bin/env python3
from smbus2 import SMBus
from time import sleep

# địa chỉ IMU bạn scan được
ADDR_ACC = 0x36   # Acc + Gyro
ADDR_CONF = 0x15
ADDR_TEMP = 0x74

bus = SMBus(1)

def read_word(addr, reg):
    hi = bus.read_byte_data(addr, reg)
    lo = bus.read_byte_data(addr, reg + 1)
    val = (hi << 8) | lo
    if val & 0x8000:  # signed
        val -= 65536
    return val

def read_accel():
    ax = read_word(ADDR_ACC, 0x01)
    ay = read_word(ADDR_ACC, 0x03)
    az = read_word(ADDR_ACC, 0x05)
    return ax, ay, az

def read_gyro():
    gx = read_word(ADDR_ACC, 0x07)
    gy = read_word(ADDR_ACC, 0x09)
    gz = read_word(ADDR_ACC, 0x0B)
    return gx, gy, gz

def read_temp():
    t = bus.read_byte_data(ADDR_TEMP, 0x00)
    return t

print("=== IMU SH3001 RAW READING ===")

while True:
    try:
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        temp = read_temp()

        print(f"ACC = {ax}, {ay}, {az} | GYRO = {gx}, {gy}, {gz} | TEMP = {temp}")
    except Exception as e:
        print("ERR:", e)

    sleep(0.1)
