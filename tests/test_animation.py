import math
import numpy as np
import time
import gpiod

from models.arm import Arm
from models.body import Body
from models.arm import Arm
from devices.servo_uart import ServoUart
from points.animation import stand_up_points
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

count = 100

arms = {
    "front_right": FrontRightArm,
    "front_left": FrontLeftArm,
    "back_right": BackRightArm,
    "back_left": BackLeftArm,
}

current_points = {
    "front_right": 0,
    "front_left": 0,
    "back_right": 0,
    "back_left": 0,
}


def get_point(leg_name, index):
    return stand_up_points[leg_name][index]


def set_arm_coord(leg_name, index):
    p = get_point(leg_name, index)
    arms[leg_name].set_coord(p["x"], p["y"], p["z"])


def set_arm_position(leg_name, index, count):
    p = get_point(leg_name, index)
    arms[leg_name].set_position(p["x"], p["y"], p["z"], count)


# Set initial positions
for leg_name in arms:
    set_arm_coord(leg_name, 0)
time.sleep(3)

# Move through all points
while True:
    finished = True

    for leg_name, arm in arms.items():
        if current_points[leg_name] >= len(stand_up_points[leg_name]) - 1:
            continue

        finished = False

        if arm.next():
            current_points[leg_name] += 1
            set_arm_position(
                leg_name,
                current_points[leg_name],
                count,
            )

    if finished:
        break

    time.sleep(0.02)