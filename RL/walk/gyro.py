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
    return (value / 32768 * 180)


def correct_gyro(value):
    return round(value, 2)


def read_word_signed(bus, address, reg):
    value = bus.read_word_data(address, reg)

    if value >= 32768:
        value -= 65536

    return value


bus = SMBus(1)

def get_angles():
    temp_raw = read_word_signed(bus, ADDRESS, TEMP_REG)
    roll_raw = read_word_signed(bus, ADDRESS, ROLL_REG)
    pitch_raw = read_word_signed(bus, ADDRESS, PITCH_REG)
    yaw_raw = read_word_signed(bus, ADDRESS, YAW_REG)
    temp = temp_raw / 100
    return correct_angle(roll_raw), correct_angle(pitch_raw), correct_angle(yaw_raw), temp

    # gx_raw = read_word_signed(bus, ADDRESS, HX_REG)
    # gy_raw = read_word_signed(bus, ADDRESS, HY_REG)
    # gz_raw = read_word_signed(bus, ADDRESS, HZ_REG)

    # temp = temp_raw / 100
    # roll = correct_angle(roll_raw)
    # pitch = correct_angle(pitch_raw)
    # yaw = correct_angle(yaw_raw)

    # gx = correct_gyro(gx_raw)
    # gy = correct_gyro(gy_raw)
    # gz = correct_gyro(gz_raw)
