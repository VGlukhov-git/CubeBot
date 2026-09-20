import math
import numpy as np
import time
import gpiod

from models.arm import Arm
from models.body import Body
from devices.servo_uart import ServoUart
from points.animation import walk_points
import threading

import socket
import struct
import numpy as np
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
    return walk_points[leg_name][index]


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

time.sleep(3)

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

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)

ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 5005

sock.bind((
    ROBOT_IP,
    ROBOT_PORT,
))

print(
    f"Listening on {ROBOT_IP}:{ROBOT_PORT}"
)

depth_values = None
can_move = False


def depth_loop():
    global depth_values
    global can_move

    while True:
        data, _ = sock.recvfrom(65535)
        time.sleep(0.01)
        rows, cols, _ = struct.unpack(
            "<HHI",
            data[:8],
        )

        depth_values = np.frombuffer(
            data[8:],
            dtype=np.float32,
        ).reshape(
            rows,
            cols,
        )

        bottom_depth = depth_values[-3, :]

        valid_depth = bottom_depth[
            np.isfinite(bottom_depth)
        ]

        if valid_depth.size > 0:
            average_depth = float(
                np.mean(valid_depth)
            )

            can_move = average_depth > -5.65
            
            
depth_thread = threading.Thread(
    target=depth_loop,
    daemon=True,
)

depth_thread.start()
can_move = True

while True:
    print(can_move)
    if can_move:
        n = [
            FrontLeftArm.next(),
            FrontRightArm.next(),
            BackLeftArm.next(),
            BackRightArm.next()
        ]
        if (n[0] == True):
            current_points[0] += 1
            if current_points[0] == len(walk_points["front_left"]):
                current_points[0] = 0
            FrontLeftArm.set_position(
                walk_points["front_left"][current_points[0]]["x"],
                walk_points["front_left"][current_points[0]]["y"],
                walk_points["front_left"][current_points[0]]["z"],
                walk_points["front_left"][current_points[0]]["count"],
                )
        if (n[1] == True):
            current_points[1] += 1
            if current_points[1] == len(walk_points["front_right"]):
                current_points[1] = 0
            FrontRightArm.set_position(
                walk_points["front_right"][current_points[1]]["x"],
                walk_points["front_right"][current_points[1]]["y"],
                walk_points["front_right"][current_points[1]]["z"],
                walk_points["front_right"][current_points[1]]["count"],
                )
        if (n[2] == True):
                current_points[2] += 1
                if current_points[2] == len(walk_points["back_left"]):
                    current_points[2] = 0
                BackLeftArm.set_position(
                    walk_points["back_left"][current_points[2]]["x"],
                    walk_points["back_left"][current_points[2]]["y"],
                    walk_points["back_left"][current_points[2]]["z"],
                    walk_points["back_left"][current_points[2]]["count"],
                    )
        if (n[3] == True):
            current_points[3] += 1
            if current_points[3] == len(walk_points["back_right"]):
                current_points[3] = 0
            BackRightArm.set_position(
                walk_points["back_right"][current_points[3]]["x"],
                walk_points["back_right"][current_points[3]]["y"],
                walk_points["back_right"][current_points[3]]["z"],
                walk_points["back_right"][current_points[3]]["count"],
                )

    time.sleep(0.01)