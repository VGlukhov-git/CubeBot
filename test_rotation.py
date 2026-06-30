import math
import numpy as np
from models.arm import Arm
from models.body import Body

from devices.servo_uart import ServoUart
import time
import gpiod

servo_device = ServoUart()
A = 78.7
B = 48
C = 25
delta_a = 21.5

FrontLeftArm = Arm({
    "chA": 3,
    "chB": 4,
    "chC": 5,
    "delta_a": delta_a - 4,
    "delta_b": 10,
    "delta_c": 0,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C
})

FrontRightArm = Arm({
    "chA": 2,
    "chB": 0,
    "chC": 1,
    "delta_a": delta_a - 3,
    "delta_b": 0,
    "delta_c": 0,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C,
    "invert": True
})

BackLeftArm = Arm({
    "chA": 11,
    "chB": 10,
    "chC": 12,
    "delta_a": delta_a,
    "delta_b": 0,
    "delta_c": 5,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C,
    "invert": True
})

BackRightArm = Arm({
    "chA": 13,
    "chB": 15,
    "chC": 14,
    "delta_a": delta_a,
    "delta_b": 3,
    "delta_c": 5,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C,
})

count = 25
Z = 60

FrontLeftArm.set_coord(45, -40 - count, Z)
FrontRightArm.set_coord(45, -40 - count, Z)
BackLeftArm.set_coord(45, -40 - count, Z)
BackRightArm.set_coord(45, -40 - count, Z)

class FakeArm:
    def __init__(self, name):
        self.name = name
        self.target = np.array([50, 50, -50])

    def set_coord(self, x, y, z):
        self.target = np.array([x, y, z])
        print(self.name, "local target:", self.target)

    def get_coord(self):
        return self.target.copy()
        
X = 92.5/2        
Y = 77/2

legs = {
    "front_left": {
        "arm": FrontLeftArm,
        "mount": np.array([X, -Y, 0]),
        "local_to_body": np.diag([1, -1, 1]),
    },
    "front_right": {
        "arm": FrontRightArm,
        "mount": np.array([X, Y, 0]),
        "local_to_body": np.diag([1, 1, 1]),
    },
    "back_left": {
        "arm": BackLeftArm,
        "mount": np.array([-X, -Y, 0]),
        "local_to_body": np.diag([-1, -1, 1]),
    },
    "back_right": {
        "arm": BackRightArm,
        "mount": np.array([-X, Y, 0]),
        "local_to_body": np.diag([-1, 1, 1]),
    },
}

body = Body(legs)
count = -25
direction = 1
time.sleep(1)

LED1 = 17
LED2 = 27
LED3 = 22

LED_PINS = [LED1, LED2, LED3]

request = gpiod.request_lines(
    "/dev/gpiochip0",
    consumer="leds",
    config={
        pin: gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT
        )
        for pin in LED_PINS
    },
)

request.set_value(17, gpiod.line.Value.ACTIVE)
request.set_value(27, gpiod.line.Value.INACTIVE)

time.sleep(0.3)
count = 0 
while True:
    phase = .5 * math.pi * count / B

    body.set_rotation(
        roll=0 * math.sin(phase),
        pitch=0 * math.sin(phase),
        yaw=25 * math.cos(phase),
    )

    count += 1
    time.sleep(1/40)
