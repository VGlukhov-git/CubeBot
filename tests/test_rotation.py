import math
import numpy as np
from models.arm import Arm
from models.body import Body

from devices.servo_uart import ServoUart
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

count = 25
Z = 60

FrontLeftArm.set_coord(45, -40 - count, Z)
FrontRightArm.set_coord(45, -40 - count, Z)
BackLeftArm.set_coord(45, -40 - count, Z)
BackRightArm.set_coord(45, -40 - count, Z)

X = 92.5/2        
Y = 77/2

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
        roll=15 * math.sin(phase),
        pitch=0 * math.sin(phase),
        yaw=0 * math.cos(phase),
    )

    count += 1
    time.sleep(1/50)
