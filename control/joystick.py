from evdev import InputDevice, list_devices, ecodes, categorize
import time
import threading
import numpy as np

from points.animation import get_walk_points, walk_points
from devices.wt901 import Wt901

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

DEVICE = "/dev/input/event5"
## Your bluethooth controller MAC address
## Connect your controller via bluetooth and run `bluetoothctl` to get the MAC address of your controller. It should look like "XX:XX:XX:XX:XX:XX"
JOYSTICK_MAC = "E4:17:D8:55:38:4B"

joystick = None

for path in list_devices():
    device = InputDevice(path)

    if (
        device.name == "Pro Controller"
        and device.uniq
        and device.uniq.upper() == JOYSTICK_MAC
    ):
        joystick = device
        break

if joystick is None:
    raise RuntimeError("Pro Controller is not connected")

print(f"Joystick: {joystick.name}")
print(f"Path: {joystick.path}")
print(f"Physical: {joystick.phys}")
print(f"Unique ID: {joystick.uniq}")
print()

# ABS_HAT0Y forward/backward
# ABS_HAT0X left/right

lastEvent = None

Z = 60

set_count = 50
rotate_direction = 1
Yaw = 35
dYaw = Yaw/3
dX = 10
dZ = 10
dY = 30
dx_speed = 0

phase = 0
delta = 1
count = 15


local_dYaw = dYaw / count
local_dy = dY / count
local_yaw = 0
last_dir = rotate_direction

should_prepare_for_rotation = True
can_rotate = False
finished = True

# for movements
can_move = False
current_points=[0,0,0,0]
K = 1.5
walk_count = 25 * K
walk_count_up = 11 * K

walk_points = get_walk_points(walk_count, walk_count_up, 0)

speed = 70

def prepare_for_movement():
    FrontLeftArm.set_position(
        walk_points["front_left"][0]["x"],
        walk_points["front_left"][0]["y"],
        walk_points["front_left"][0]["z"],
        walk_points["front_left"][0]["count"],
    )
    FrontRightArm.set_position(
        walk_points["front_right"][0]["x"],
        walk_points["front_right"][0]["y"],
        walk_points["front_right"][0]["z"],
        walk_points["front_right"][0]["count"],
    )
    BackLeftArm.set_position(
        walk_points["back_left"][0]["x"],
        walk_points["back_left"][0]["y"],
        walk_points["back_left"][0]["z"],
        walk_points["back_left"][0]["count"],
    )
    BackRightArm.set_position(
        walk_points["back_right"][0]["x"],
        walk_points["back_right"][0]["y"],
        walk_points["back_right"][0]["z"],
        walk_points["back_right"][0]["count"],
    )

def set_initial_position():
    FrontLeftArm.set_position(45, -40 - 35, Z, set_count)
    FrontRightArm.set_position(45, -40 - 35, Z, set_count)
    BackLeftArm.set_position(45, -40 - 35, Z, set_count)
    BackRightArm.set_position(45, -40 - 35, Z, set_count)
    is_all_legs_ready = False
    
    while not is_all_legs_ready:
        is_all_legs_ready = all([
            FrontLeftArm.next(),
            FrontRightArm.next(),
            BackLeftArm.next(),
            BackRightArm.next(),
        ])
        time.sleep(0.01)
        
def set_bottom_position():
    Z = 20
    FrontLeftArm.set_position(0, -40, Z, set_count)
    FrontRightArm.set_position(0, -40, Z, set_count)
    BackLeftArm.set_position(0, -40, Z, set_count)
    BackRightArm.set_position(0, -40, Z, set_count)
    is_all_legs_ready = False
    
    while not is_all_legs_ready:
        is_all_legs_ready = all([
            FrontLeftArm.next(),
            FrontRightArm.next(),
            BackLeftArm.next(),
            BackRightArm.next(),
        ])
        time.sleep(0.01)

set_initial_position()

## Preparation before rotation
def prepare_for_rotation():
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

PHASE_LEGS = [
    "front_right",
    "back_right",
    "back_left",
    "front_left",
]

# 315 - start
# 314 - select
# 307 - X
# 305 - A
# 304 - B
# 308 - Y

def joystick_loop():
    global lastEvent
    global rotate_direction
    global can_rotate
    global should_prepare_for_rotation
    global phase
    global local_yaw
    global finished
    global last_dir
    global can_move
    global current_points
    global speed
    global dx_speed

    isSet = False
    roll = 0
    pitch = 0
    yaw = 0
    def set_rotation():
        # should_prepare_for_rotation = True
        print(body.yaw)
        body.set_rotation(
                roll=roll,
                pitch=pitch,
                yaw=yaw + body.yaw,
            )
    for event in joystick.read_loop():
        # Buttons
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)

            if key_event.keystate == key_event.key_down:
                print(f"BUTTON {event.code}: PRESSED")
                if event.code == 304:
                    set_bottom_position()
                if event.code == 307:
                    can_rotate = False
                    finished = True

                    phase = 0
                    local_yaw = 0.0
                    last_dir = rotate_direction

                    roll = 0
                    pitch = 0
                    yaw = 0

                    should_prepare_for_rotation = True

                    body.reset_state()
                    set_initial_position()

            elif key_event.keystate == key_event.key_up:
                print(f"BUTTON {event.code}: RELEASED")

        # Analog sticks / triggers / D-pad
        elif event.type == ecodes.EV_ABS:
            axis_name = ecodes.ABS.get(event.code, f"ABS_{event.code}")
            lastEvent = axis_name
            
            if axis_name == 'ABS_X':
                # roll = min(20, -event.value / 32000 * 20)
                dx_speed = event.value / 32000 * 40
                # set_rotation()
            if axis_name == 'ABS_RY':
                pitch = min(20, event.value / 32000 * 30)    
                set_rotation()
            if axis_name == 'ABS_RX':
                yaw = min(20, event.value / 32000 * 30)
                set_rotation()
            if axis_name == 'ABS_Y':
                if event.value > -500:
                    can_move = False
                    if isSet:                    
                        can_rotate = False
                        finished = True
                        phase = 0
                        local_yaw = 0.0
                        last_dir = rotate_direction
                        roll = 0
                        pitch = 0
                        yaw = 0
                        should_prepare_for_rotation = True
                        body.reset_state()
                        set_initial_position()
                        isSet = False
                        
                if event.value < -500:
                    if not isSet:
                        prepare_for_movement()
                        current_points = [0,0,0,0]
                        should_prepare_for_rotation = True
                        can_move = True
                        isSet = True
                # speed = abs(int(event.value / 32000 * 100))
                # print(f"Speed: {speed}")

            # if axis_name == 'ABS_HAT0Y':
            #     if event.value == 0:
            #         can_move = False
            #         set_initial_position()
                    
            #     if event.value == -1:
            #         prepare_for_movement()
            #         current_points = [0,0,0,0]
            #         should_prepare_for_rotation = True
            #         can_move = True

            if axis_name == 'ABS_HAT0X':
                speed = 60
                if event.value == 0:
                    can_rotate = False
                if event.value == 1:
                    can_rotate = True
                    rotate_direction = 1
                    if should_prepare_for_rotation:
                        last_dir = rotate_direction
                        should_prepare_for_rotation = False
                        prepare_for_rotation()
                if event.value == -1:
                    can_rotate = True
                    rotate_direction = -1
                    if should_prepare_for_rotation:
                        phase = 7
                        last_dir = rotate_direction
                        should_prepare_for_rotation = False
                        prepare_for_rotation()

           
            print(
                f"AXIS {axis_name} ({event.code}): {event.value}"
            )

def imu_loop():
    imu = Wt901()
    last_roll = 0
    last_pitch = 0
    delta = 2
    while True:
        roll = imu.get_roll() + 180
        pitch = imu.get_pitch() + 180
        yaw = imu.get_yaw()
        temp = imu.get_temp()
        print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Temp: {temp:.2f}°C")
        if abs(roll) > delta:
            last_roll = roll/11
            body.set_rotation_local(
                roll=last_roll,
                pitch=0,
                yaw=0,
            )
            time.sleep(0.01)

        if abs(pitch) > delta:
            last_pitch = pitch/11
            body.set_rotation_local(
                roll=last_roll,
                pitch=-last_pitch,
                yaw=0,
            )
            time.sleep(0.01)
        time.sleep(0.01)

joystick_thread = threading.Thread(
    target=joystick_loop,
    daemon=True,
)


imu_thread = threading.Thread(
    target=imu_loop,
    daemon=True,
)

joystick_thread.start()
# imu_thread.start()

while True:
    ### MOVEMENT
    if can_move:
        walk_points = get_walk_points(walk_count, walk_count_up, dx_speed)
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

    ### ROTATION
    if can_rotate == True or not finished:
        local_dYaw = dYaw / count
        revert = False
        should_make_revert = rotate_direction != last_dir
        if (should_make_revert):
            last_dir = rotate_direction
            phase += 1 * rotate_direction
            if phase > 7:
                phase = 0
            if phase < 0:
                phase = 7
        if phase == 0:
            body.front_right.move(y=local_dy* rotate_direction)
            # return the leg back to the initial position
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=-3*local_dYaw * rotate_direction,
                exclude_legs=["front_left", "back_left", "back_right"],
            )
            # rotate full body
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=local_dYaw * rotate_direction,
                exclude_legs=["front_right"],
            )
        if phase == 1:
            body.front_right.move(y=-local_dy* rotate_direction)
            
        if phase == 2:
            body.back_right.move(y=local_dy* rotate_direction)
            # return the leg back to the initial position
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=-3*local_dYaw * rotate_direction,
                exclude_legs=["front_left", "back_left", "front_right"],
            )
            # rotate full body
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=local_dYaw * rotate_direction,
                exclude_legs=["back_right"],
            )
        if phase == 3:
            body.back_right.move(y=-local_dy* rotate_direction)

        if phase == 4:
            body.back_left.move(y=local_dy* rotate_direction)
            # return the leg back to the initial position
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=-3*local_dYaw * rotate_direction,
                exclude_legs=["front_left", "back_right", "front_right"],
            )
            # rotate full body
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=local_dYaw * rotate_direction,
                exclude_legs=["back_left"],
            )
        if phase == 5:
            body.back_left.move(y=-local_dy* rotate_direction)
        
        if phase == 6:
            body.front_left.move(y=local_dy* rotate_direction)
            # return the leg back to the initial position
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=-3*local_dYaw* rotate_direction,
                exclude_legs=["back_left", "back_right", "front_right"],
            )
            # rotate full body
            body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=local_dYaw* rotate_direction,
                exclude_legs=["front_left"],
            )
        if phase == 7:
            body.front_left.move(y=-local_dy* rotate_direction) 

        if local_yaw <= dYaw:
            local_yaw += local_dYaw
            finished = False
        if local_yaw > dYaw:
            phase += 1 * rotate_direction
            finished = True
            if phase > 7:
                phase = 0
            if phase < 0:
                phase = 7
            local_yaw = 0

    time.sleep(1/(speed + 40))