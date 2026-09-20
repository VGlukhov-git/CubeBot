import math
import numpy as np
import time
import gpiod

from models.arm import Arm
from models.body import Body
from devices.servo_uart import ServoUart
from points.animation import rotate_right_points 
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


X = 92.5 / 2
Y = 77 / 2

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
    return rotate_right_points[leg_name][index]


def set_arm_coord(leg_name, index):
    p = get_point(leg_name, index)
    arms[leg_name].set_coord(
        p["x"],
        p["y"],
        p["z"],
    )


def set_arm_position(leg_name, index):
    p = get_point(leg_name, index)

    arms[leg_name].set_position(
        p["x"],
        p["y"],
        p["z"],
        p["count"],
    )


# ------------------------------------------------------------------
# Move robot to initial pose
# ------------------------------------------------------------------

for leg_name in arms:
    set_arm_coord(leg_name, 0)

for leg_name in arms:
    current_points[leg_name] = 0
    arms[leg_name].set_coord(
        0,
        -A,
        B + C,
    )

time.sleep(0.1)

# ------------------------------------------------------------------
# Start first movement
# ------------------------------------------------------------------

for leg_name in arms:
    current_points[leg_name] = 0
    set_arm_position(
        leg_name,
    current_points[leg_name])

current_points=[0,0,0,0]

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

while True:
    # body.set_rotation(
    #     roll=0,
    #     pitch=0,
    #     yaw=0
    # )
    n = [
        FrontLeftArm.next(),
        FrontRightArm.next(),
        BackLeftArm.next(),
        BackRightArm.next()
    ]
    print(n)
    if (n[0] == True):
        current_points[0] += 1
        if current_points[0] == len(rotate_right_points["front_left"]):
            current_points[0] = 0
        FrontLeftArm.set_position(
            rotate_right_points["front_left"][current_points[0]]["x"],
            rotate_right_points["front_left"][current_points[0]]["y"],
            rotate_right_points["front_left"][current_points[0]]["z"],
            rotate_right_points["front_left"][current_points[0]]["count"],
            )
    if (n[1] == True):
        current_points[1] += 1
        if current_points[1] == len(rotate_right_points["front_right"]):
            current_points[1] = 0
        FrontRightArm.set_position(
            rotate_right_points["front_right"][current_points[1]]["x"],
            rotate_right_points["front_right"][current_points[1]]["y"],
            rotate_right_points["front_right"][current_points[1]]["z"],
            rotate_right_points["front_right"][current_points[1]]["count"],
            )
    if (n[2] == True):
            current_points[2] += 1
            if current_points[2] == len(rotate_right_points["back_left"]):
                current_points[2] = 0
            BackLeftArm.set_position(
                rotate_right_points["back_left"][current_points[2]]["x"],
                rotate_right_points["back_left"][current_points[2]]["y"],
                rotate_right_points["back_left"][current_points[2]]["z"],
                rotate_right_points["back_left"][current_points[2]]["count"],
                )
    if (n[3] == True):
        current_points[3] += 1
        if current_points[3] == len(rotate_right_points["back_right"]):
            current_points[3] = 0
        BackRightArm.set_position(
            rotate_right_points["back_right"][current_points[3]]["x"],
            rotate_right_points["back_right"][current_points[3]]["y"],
            rotate_right_points["back_right"][current_points[3]]["z"],
            rotate_right_points["back_right"][current_points[3]]["count"],
            )

    time.sleep(0.002)