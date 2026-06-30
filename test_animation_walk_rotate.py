import math
import numpy as np
import time
import gpiod

from models.arm import Arm
from models.body import Body
from devices.servo_uart import ServoUart
from points.animation import walk_points


class LegAnimator:
    def __init__(self, points):
        self.points = points
        self.index = 0

        p = self.points[0]

        self.x = p["x"]
        self.y = p["y"]
        self.z = p["z"]

        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.count = 0

        self.start_next_point()

    def start_next_point(self):
        self.index += 1

        if self.index >= len(self.points):
            self.index = 0

        p = self.points[self.index]
        steps = max(1, p["count"])

        self.dx = (p["x"] - self.x) / steps
        self.dy = (p["y"] - self.y) / steps
        self.dz = (p["z"] - self.z) / steps

        self.count = steps

    def next(self):
        if self.count <= 0:
            self.start_next_point()

        self.x += self.dx
        self.y += self.dy
        self.z += self.dz

        self.count -= 1

        return self.x, self.y, self.z


servo_device = ServoUart()

A = 82.8 + 5
B = 48
C = 25
delta_a = 25

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

FrontLeftArm = Arm({
    "chA": 3,
    "chB": 4,
    "chC": 5,
    "delta_a": delta_a - 4,
    "delta_b": 10,
    "delta_c": 0,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C
})

FrontRightArm = Arm({
    "chA": 2,
    "chB": 0,
    "chC": 1,
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
    "chA": 11,
    "chB": 10,
    "chC": 12,
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
    "chA": 13,
    "chB": 15,
    "chC": 14,
    "delta_a": delta_a,
    "delta_b": 3,
    "delta_c": 5,
    "device": servo_device,
    "A": A,
    "B": B,
    "C": C,
})

X = 92.5 / 2
Y = 77 / 2

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

arms = {
    "front_left": FrontLeftArm,
    "front_right": FrontRightArm,
    "back_left": BackLeftArm,
    "back_right": BackRightArm,
}

animations = {
    "front_left": LegAnimator(walk_points["front_left"]),
    "front_right": LegAnimator(walk_points["front_right"]),
    "back_left": LegAnimator(walk_points["back_left"]),
    "back_right": LegAnimator(walk_points["back_right"]),
}

# ------------------------------------------------------------------
# Move robot to initial pose
# ------------------------------------------------------------------

for leg_name, arm in arms.items():
    p = walk_points[leg_name][0]
    arm.set_coord(
        p["x"],
        p["y"],
        p["z"],
    )

time.sleep(1)

# ------------------------------------------------------------------
# Walking loop
# ------------------------------------------------------------------

frame = 0

while True:
    phase = frame * 0.05

    # Optional body rotation
    # Set all to 0 if you want only walking.
    body.set_rotation_matrix(
        roll=0,
        pitch=6,
        yaw=0,
    )

    for leg_name, animation in animations.items():
        x, y, z = animation.next()

        # Without body rotation:
        # arms[leg_name].set_coord(x, y, z)

        # With body transform:
        body.set_leg_coord(leg_name, x, y, z)

    frame += 1
    time.sleep(0.005)