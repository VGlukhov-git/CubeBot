import math
import numpy as np

from models.arm import Arm
from models.body import Body

from devices.servo_uart import ServoUart
from devices.wt901 import Wt901

import time
import gpiod

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

servo_device = ServoUart()
A = 78.7
B = 48
C = 25
delta_a = 21.5

FrontLeftArm = Arm({
    "chA": 3,
    "chB": 4,
    "chC": 5,
    "delta_a": delta_a,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C
})

FrontRightArm = Arm({
    "chA": 2,
    "chB": 0,
    "chC": 1,
    "delta_a": delta_a,
    "delta_b": -5,
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
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C,
})

# point = [x,y,z]
points = [[], [], [], []]

        
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

count = 15
Z = 60

FrontLeftArm.set_coord(55, -40 - count, Z)
FrontRightArm.set_coord(55, -40 - count, Z)
BackLeftArm.set_coord(55, -40 - count, Z)
BackRightArm.set_coord(55, -40 - count, Z)

body = Body(legs)
wt901 = Wt901()


body.set_rotation(0, 0, 0)
time.sleep(1)

count = 0 
fixed = True
last_roll = 0
last_pitch = 0
delta = 0.3

request.set_value(17, gpiod.line.Value.ACTIVE)
request.set_value(27, gpiod.line.Value.ACTIVE)

while True:
    request.set_value(17, gpiod.line.Value.ACTIVE)
    request.set_value(27, gpiod.line.Value.ACTIVE)
    roll = wt901.get_roll() + 180 + 3.7
    pitch = wt901.get_pitch() + 180 
    print(round(pitch, 2), round(roll, 2))
    if abs(roll) > delta:
        request.set_value(17, gpiod.line.Value.INACTIVE)
        last_roll += roll/3
        body.set_rotation(
            roll=last_roll,
            pitch=-last_pitch,
            yaw=0,
        )
        time.sleep(0.003)

    if abs(pitch) > delta:
        last_pitch += pitch/3
        request.set_value(27, gpiod.line.Value.INACTIVE)
        body.set_rotation(
            roll=last_roll,
            pitch=-last_pitch,
            yaw=0,
        )
        time.sleep(0.003)
        
    time.sleep(1/40)