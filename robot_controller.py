import time
import gpiod
import json
import socket
import math

import threading
import ai.ai_camera as ai_camera

import numpy as np
from points.animation import walk_points

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

# ---------------------------------------------------------
# LEDs
# ---------------------------------------------------------

LED1 = 17
LED2 = 27
LED3 = 22

LED_PINS = [
    LED1,
    LED2,
    LED3,
]

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

# ---------------------------------------------------------
# Move to standing position
# ---------------------------------------------------------

for leg_name in arms:
    arms[
        leg_name
    ].set_coord(
        50,
        -A + 10,
        B + C - 20,
    )
    time.sleep(0.2)


time.sleep(1)


# ---------------------------------------------------------
# Start first animation movement
# ---------------------------------------------------------

# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------


UNITY_IP = "192.168.1.6"
UNITY_PORT = 5006

unity_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)


def send_state_to_unity():
    state = body.get_arms_state()

    message = {
        "type": "robot_state",
        "state": state,
    }

    data = json.dumps(
        message,
        separators=(",", ":"),
    ).encode("utf-8")

    unity_socket.sendto(
        data,
        (
            UNITY_IP,
            UNITY_PORT,
        ),
    )

count = 0 
# start camera + depth estimation
ai_camera_thread = threading.Thread(
    target=ai_camera.camera_loop,
    daemon=True,
    name="camera_loop",
)

ai_camera_thread.start()

# Walking

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

# for leg_name in arms:
#     set_arm_coord(leg_name, 0)

# for leg_name in arms:
#     current_points[leg_name] = 0
   
    
# for leg_name in arms:
#     current_points[leg_name] = 0
#     set_arm_position(
#         leg_name,
#     current_points[leg_name])

current_points=[0,0,0,0]
        
while True:
    request.set_value(
        LED1,
        gpiod.line.Value.ACTIVE,
    )

    # -----------------------------------------
    # Depth
    # -----------------------------------------

    depth_values = ai_camera.shared_depth_value

    can_move = False
    average_depth = np.nan

    if depth_values is not None:
        # Last row
        bottom_depth = depth_values[-1:, :]

        valid_depth = bottom_depth[
            np.isfinite(bottom_depth)
        ]

        if valid_depth.size > 0:
            average_depth = float(
                np.mean(valid_depth)
            )

            can_move = average_depth > -5.6

    print(
        "Bottom average depth:",
        average_depth,
        "can move:",
        can_move,
    )

    # -----------------------------------------
    # Walking
    # -----------------------------------------

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

    # -----------------------------------------
    # Unity
    # -----------------------------------------

    send_state_to_unity()

    request.set_value(
        LED1,
        gpiod.line.Value.INACTIVE,
    )

    time.sleep(0.002)