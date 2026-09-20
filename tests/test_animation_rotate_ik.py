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
        40,
        -A,
        B + C - 20,
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

Yaw = 40

dYaw = Yaw/3

dX = 10
dZ = 10
dY = 30
#
#
#
body.move_local(
    x=0,
    y=0,
    z=0,
)

phase = 0

local_dy = 0

delta = 1
count = 100

print(dYaw)
# front left
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=-dYaw * 1.5,
    exclude_legs=["front_right", "back_left", "back_right"],
)

# front right
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=dYaw * 1.5,
    exclude_legs=["front_left", "back_left", "back_right"],
)
# back left
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=dYaw * -0.5,
    exclude_legs=["front_left", "front_right", "back_right"],
)
# # back right
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=dYaw * 0.5,
    exclude_legs=["front_left", "front_right", "back_left"],
)      

local_dYaw = dYaw / count
local_dy = dY / count
local_yaw = 0
while True:
    print (phase, local_yaw, dYaw)
    if phase == 0:
        body.front_right.move(y=local_dy)
        # return the leg back to the initial position
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=-3*local_dYaw,
            exclude_legs=["front_left", "back_left", "back_right"],
        )
        # rotate full body
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=local_dYaw,
            exclude_legs=["front_right"],
        )
    if phase == 1:
        body.front_right.move(y=-local_dy)
        
    if phase == 2:
        body.back_right.move(y=local_dy)
        # return the leg back to the initial position
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=-3*local_dYaw,
            exclude_legs=["front_left", "back_left", "front_right"],
        )
        # rotate full body
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=local_dYaw,
            exclude_legs=["back_right"],
        )
    if phase == 3:
        body.back_right.move(y=-local_dy)

    if phase == 4:
        body.back_left.move(y=local_dy)
        # return the leg back to the initial position
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=-3*local_dYaw,
            exclude_legs=["front_left", "back_right", "front_right"],
        )
        # rotate full body
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=local_dYaw,
            exclude_legs=["back_left"],
        )
    if phase == 5:
        body.back_left.move(y=-local_dy)
    
    if phase == 6:
        body.front_left.move(y=local_dy)
        # return the leg back to the initial position
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=-3*local_dYaw,
            exclude_legs=["back_left", "back_right", "front_right"],
        )
        # rotate full body
        body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=local_dYaw,
            exclude_legs=["front_left"],
        )
    if phase == 7:
        body.front_left.move(y=-local_dy) 

    if local_yaw <= dYaw:
        local_yaw += local_dYaw
    if local_yaw > dYaw:
        phase += 1
        if phase > 7:
            phase = 0
        local_yaw = 0
        

    time.sleep(0.001)