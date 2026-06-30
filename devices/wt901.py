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


class Wt901:
    def __init__(self):
        print("Init wt901")
        self.bus = SMBus(1)
    def get_roll(self):
        return correct_angle(read_word_signed(self.bus, ADDRESS, ROLL_REG))
    def get_pitch(self):
        return correct_angle(read_word_signed(self.bus, ADDRESS, PITCH_REG))
        