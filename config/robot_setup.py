import numpy as np

from models.arm import Arm
from models.body import Body
from devices.servo_uart import ServoUart
import json

# ---------------------------------------------------------
# Settings
# ---------------------------------------------------------

SETTINGS_FILE = Path(__file__).resolve().parent / "settings.json"
CALIBRATION_FILE = BASE_DIR / "calibration.json"

def load_json(path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)

mapper = load_json(SETTINGS_FILE)["mapper"]
calibration = load_json(CALIBRATION_FILE)

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

def create_arm(name, invert=False):
    channels = mapper[name]
    offsets = calibration[name]

    return Arm({
        "chA": channels["hip"],
        "chB": channels["upperLeg"],
        "chC": channels["knee"],
        "delta_a": delta_a + offsets["delta_a"],
        "delta_b": offsets["delta_b"],
        "delta_c": offsets["delta_c"],
        "device": servo_device,
        "A": A,
        "B": B,
        "C": C,
        "invert": invert
    })


FrontLeftArm = create_arm("front-left")
FrontRightArm = create_arm("front-right", invert=True)
BackLeftArm = create_arm("rear-left", invert=True)
BackRightArm = create_arm("rear-right")

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