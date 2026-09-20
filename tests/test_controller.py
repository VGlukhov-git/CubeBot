from models.arm import Arm
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

delta_a = 25
# delta_a = 0

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
    

# FrontLeftArm.set_angle(90, 90, 180)
# FrontRightArm.set_angle(0, 140, 90)
# BackLeftArm.set_angle(0, 140, 90)
# BackRightArm.set_angle(0, 140, 90)
count = 0
# while count < 50:
    # count += 0.1
    # time.sleep(0.01)
# FrontLeftArm.set_coord(B, -A, 0)
# FrontLeftArm.set_angle(90, 90, 180)
# FrontLeftArm.set_coord(1, -40, 0)
# time.sleep(1)
count = 35

Z = 60
# FrontLeftArm.set_coord(count, -40, Z)
# FrontRightArm.set_coord(count, -40, Z)
# BackLeftArm.set_coord(count, -40, Z)
# BackRightArm.set_coord(count, -40, Z)

request.set_value(17, gpiod.line.Value.ACTIVE)
request.set_value(27, gpiod.line.Value.INACTIVE)
request.set_value(22, gpiod.line.Value.INACTIVE)
time.sleep(1)

request.set_value(17, gpiod.line.Value.INACTIVE)
request.set_value(27, gpiod.line.Value.ACTIVE)
request.set_value(22, gpiod.line.Value.INACTIVE)
time.sleep(1)

request.set_value(17, gpiod.line.Value.INACTIVE)
request.set_value(27, gpiod.line.Value.INACTIVE)
request.set_value(22, gpiod.line.Value.ACTIVE)
time.sleep(1)
 
# while count < 35:
#     FrontLeftArm.set_coord(count, -40, Z)
#     FrontRightArm.set_coord(count, -40, Z)
#     BackLeftArm.set_coord(count, -40, Z)
#     BackRightArm.set_coord(count, -40, Z)
#     count += 1
#     time.sleep(0.01)

count = 0
time.sleep(0.3)

while count < B:
    FrontLeftArm.set_coord(55, -40 - count, Z)
    FrontRightArm.set_coord(55, -40 - count, Z)
    BackLeftArm.set_coord(55, -40 - count, Z)
    BackRightArm.set_coord(55, -40 - count, Z)

    count += 1
    time.sleep(0.1)