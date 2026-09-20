import numpy as np

from models.arm import Arm
from models.body import Body
from devices.servo_uart import ServoUart


# ---------------------------------------------------------
# Servo device
# ---------------------------------------------------------

servo_device = ServoUart()

# ---------------------------------------------------------
# Arm geometry
# ---------------------------------------------------------

A = 82.8 + 10
B = 48
C = 25

delta_a = 21.5

# ---------------------------------------------------------
# Body geometry
# ---------------------------------------------------------

BODY_X = 92.5 / 2
BODY_Y = 77 / 2

# ---------------------------------------------------------
# Arms
# ---------------------------------------------------------

FrontLeftArm = Arm({
    "chA": 1,
    "chB": 2,
    "chC": 0,
    "delta_a": delta_a - 4,
    "delta_b": 10,
    "delta_c": 0,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C
})

FrontRightArm = Arm({
    "chA": 3,
    "chB": 4,
    "chC": 5,
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
    "chA": 8,
    "chB": 7,
    "chC": 6,
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
    "chA": 9,
    "chB": 10,
    "chC": 11,
    "delta_a": delta_a,
    "delta_b": 3,
    "delta_c": 5,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C,
})


# ---------------------------------------------------------
# Arms dictionary
# ---------------------------------------------------------

arms = {
    "front_left": FrontLeftArm,
    "front_right": FrontRightArm,
    "back_left": BackLeftArm,
    "back_right": BackRightArm,
}


# ---------------------------------------------------------
# Legs / mounting configuration
# ---------------------------------------------------------

legs = {
    "front_left": {
        "arm": FrontLeftArm,

        "mount": np.array([
            BODY_X,
            -BODY_Y,
            0,
        ], dtype=float),

        "local_to_body": np.diag([
            1,
            -1,
            1,
        ]),
    },

    "front_right": {
        "arm": FrontRightArm,

        "mount": np.array([
            BODY_X,
            BODY_Y,
            0,
        ], dtype=float),

        "local_to_body": np.diag([
            1,
            1,
            1,
        ]),
    },

    "back_left": {
        "arm": BackLeftArm,

        "mount": np.array([
            -BODY_X,
            -BODY_Y,
            0,
        ], dtype=float),

        "local_to_body": np.diag([
            -1,
            -1,
            1,
        ]),
    },

    "back_right": {
        "arm": BackRightArm,

        "mount": np.array([
            -BODY_X,
            BODY_Y,
            0,
        ], dtype=float),

        "local_to_body": np.diag([
            -1,
            1,
            1,
        ]),
    },
}


# ---------------------------------------------------------
# Body
# ---------------------------------------------------------

body = Body(legs)

# ---------------------------------------------------------
# Initial animation state
# ---------------------------------------------------------

current_points = {
    "front_left": 0,
    "front_right": 0,
    "back_left": 0,
    "back_right": 0,
}