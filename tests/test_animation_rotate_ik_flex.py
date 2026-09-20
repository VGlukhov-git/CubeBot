import time
import gpiod

from points.animation import rotate_right_points
from config.robot_setup import (
    A,
    B,
    C,
    arms,
    body,
    current_points,
)

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
# Initial pose
# ---------------------------------------------------------

for leg_name in arms:
    arms[leg_name].set_coord(
        50,
        -A + 10,
        B + C - 20,
    )


time.sleep(0.1)


# ---------------------------------------------------------
# Start first animation movement
# ---------------------------------------------------------


# ---------------------------------------------------------
# Rotation parameters
# ---------------------------------------------------------

YAW = 50

dYaw = YAW / 3

dX = 10
dY = 50
dZ = 10

count = 100

local_dYaw = dYaw / count
local_dy = dY / count

KY = 2.03

direction = 1
phase = 0
local_yaw = 0.0


# ---------------------------------------------------------
# Optional body offset
# ---------------------------------------------------------

body.move_local(
    x=0,
    y=0,
    z=0,
)


# ---------------------------------------------------------
# Initial leg orientation
# ---------------------------------------------------------

# Front left
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=-dYaw * 1.5,
    exclude_legs=[
        "front_right",
        "back_left",
        "back_right",
    ],
)

# Front right
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=dYaw * 1.5,
    exclude_legs=[
        "front_left",
        "back_left",
        "back_right",
    ],
)

# Back left
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=-dYaw * 0.5,
    exclude_legs=[
        "front_left",
        "front_right",
        "back_right",
    ],
)

# Back right
body.set_rotation_local(
    roll=0,
    pitch=0,
    yaw=dYaw * 0.5,
    exclude_legs=[
        "front_left",
        "front_right",
        "back_left",
    ],
)


print("dYaw:", dYaw)
print("local_dy:", local_dy)

time.sleep(1)


# ---------------------------------------------------------
# Phase configuration
# ---------------------------------------------------------

PHASE_LEGS = [
    "front_right",
    "back_right",
    "back_left",
    "front_left",
]


# ---------------------------------------------------------
# Main gait loop
# ---------------------------------------------------------

while True:
    active_leg = PHASE_LEGS[phase]

    print(
        "phase:",
        phase,
        "leg:",
        active_leg,
        "yaw:",
        local_yaw,
    )

    # -----------------------------------------------------
    # Move active leg
    # -----------------------------------------------------

    if local_yaw < dYaw * 2 / 3:
        getattr(
            body,
            active_leg,
        ).move(
            y=local_dy,
        )

    else:
        getattr(
            body,
            active_leg,
        ).move(
            y=-KY * local_dy,
        )

    # -----------------------------------------------------
    # Rotate active leg back relative to body
    # -----------------------------------------------------

    exclude_legs = [
        leg_name
        for leg_name in PHASE_LEGS
        if leg_name != active_leg
    ]

    body.set_rotation_local(
        roll=0,
        pitch=0,
        yaw=-3 * local_dYaw * direction,
        exclude_legs=exclude_legs,
    )

    # -----------------------------------------------------
    # Rotate body while keeping active leg fixed
    # -----------------------------------------------------

    body.set_rotation_local(
        roll=0,
        pitch=0,
        yaw=local_dYaw * direction,
        exclude_legs=[
            active_leg,
        ],
    )

    # -----------------------------------------------------
    # Update phase progress
    # -----------------------------------------------------

    local_yaw += abs(local_dYaw)

    if local_yaw >= dYaw:
        phase += direction

        if phase > 3:
            phase = 0

        elif phase < 0:
            phase = 3

        local_yaw = 0.0

    time.sleep(0.003)