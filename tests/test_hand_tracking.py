import math
import numpy as np
import threading

from models.arm import Arm
from models.body import Body

from devices.servo_uart import ServoUart
import time
import gpiod

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

from ai.hand_tracking import HandTracker

count = 25
Z = 60

FrontLeftArm.set_coord(45, -40 - count, Z)
FrontRightArm.set_coord(45, -40 - count, Z)
BackLeftArm.set_coord(45, -40 - count, Z)
BackRightArm.set_coord(45, -40 - count, Z)

class FakeArm:
    def __init__(self, name):
        self.name = name
        self.target = np.array([50, 50, -50])

    def set_coord(self, x, y, z):
        self.target = np.array([x, y, z])
        print(self.name, "local target:", self.target)

    def get_coord(self):
        return self.target.copy()
        
X = 92.5/2        
Y = 77/2

count = -25
direction = 1
time.sleep(1)

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

request.set_value(17, gpiod.line.Value.ACTIVE)
request.set_value(27, gpiod.line.Value.INACTIVE)

time.sleep(0.3)
count = 0

hand_tracker = HandTracker()

target_x = 300
target_y = 200

target_pitch = 0
target_yaw = 0

delta_box = 10

# def hand_tracker_worker():
while True:
    body.set_rotation(
        roll=0,
        pitch=target_pitch,
        yaw=target_yaw,
    )
    hands = hand_tracker.get_last_hands()
    # print(hands)
    if hands:
        cx, cy = hands[0]["center"]
        print(f"Center: x={cx}, y={cy}")
        delta_y = abs(cy - target_y) / (180*3)
        delta_x = abs(cx - target_x) / (150*3)
        if (cy > target_y - delta_box):
            target_pitch += delta_y
        if (cy < target_y + delta_box):
            target_pitch -= delta_y
        if (cx > target_x - delta_box):
            target_yaw += delta_x
        if (cx < target_x + delta_box):
            target_yaw -= delta_x
    else:
        print("No hands detected")
    time.sleep(0.003)

# hand_tracker_worker()

# threading.Thread(
#         target=hand_tracker_worker,
#         daemon=True,
#     ).start()