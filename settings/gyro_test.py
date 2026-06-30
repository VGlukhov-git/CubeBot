import time
from smbus2 import SMBus

ADDRESS = 0x50

TEMP_REG = 0x40
ROLL_REG = 0x3D
PITCH_REG = 0x3E
YAW_REG = 0x3F

HX_REG = 0x3A
HY_REG = 0x3B
HZ_REG = 0x3C


def correct_angle(value):
    return (value / 32768 * 180) - 180


def correct_gyro(value):
    return round(value, 2)


def read_word_signed(bus, address, reg):
    value = bus.read_word_data(address, reg)

    # Convert unsigned 16-bit to signed 16-bit
    if value >= 32768:
        value -= 65536

    return value


with SMBus(1) as bus:
    while True:
        try:
            temp_raw = read_word_signed(bus, ADDRESS, TEMP_REG)
            roll_raw = read_word_signed(bus, ADDRESS, ROLL_REG)
            pitch_raw = read_word_signed(bus, ADDRESS, PITCH_REG)
            yaw_raw = read_word_signed(bus, ADDRESS, YAW_REG)

            # gx_raw = read_word_signed(bus, ADDRESS, HX_REG)
            # gy_raw = read_word_signed(bus, ADDRESS, HY_REG)
            # gz_raw = read_word_signed(bus, ADDRESS, HZ_REG)

            temp = temp_raw / 100
            roll = correct_angle(roll_raw)
            pitch = correct_angle(pitch_raw)
            yaw = correct_angle(yaw_raw)

            # gx = correct_gyro(gx_raw)
            # gy = correct_gyro(gy_raw)
            # gz = correct_gyro(gz_raw)
            print(f"Temp : {temp:7.3f}°")
            print(f"Roll : {roll:7.3f}°")
            print(f"Pitch: {pitch:7.3f}°")
            print(f"Yaw  : {yaw:7.3f}°")
            print("-" * 20)

            time.sleep(0.001)

        except KeyboardInterrupt:
            break

        except Exception as e:
            print("I2C error:", e)
            time.sleep(0.5)