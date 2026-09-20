import math
import numpy as np

from models.arm import Arm
from models.body import Body

from devices.servo_uart import ServoUart
from devices.wt901 import Wt901

import time
import gpiod

from config.robot_setup import (
    A,
    B,
    C,
    arms,
    body,
    FrontLeftArm,
    FrontRightArm,
    BackLeftArm,
    BackRightArm
)

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

        
X = 92.5/2        
Y = 77/2

count = 15
Z = 60

FrontLeftArm.set_coord(55, -40 - count, Z)
FrontRightArm.set_coord(55, -40 - count, Z)
BackLeftArm.set_coord(55, -40 - count, Z)
BackRightArm.set_coord(55, -40 - count, Z)

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
        
    time.sleep(1/50)