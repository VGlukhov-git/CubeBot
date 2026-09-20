import math
import numpy as np
import time
import gpiod

from models.arm import Arm
from models.body import Body
from devices.servo_uart import ServoUart
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

    body.set_rotation_matrix(
        roll=0,
        pitch=15,
        yaw=0,
    )

    for leg_name, animation in animations.items():
        x, y, z = animation.next()
        body.set_leg_coord(leg_name, x, y, z)

    frame += 1
    time.sleep(0.005)