from evdev import InputDevice, ecodes, categorize
import time
import threading

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

joystick = InputDevice(DEVICE)

print(f"Joystick: {joystick.name}")
print(f"Path: {joystick.path}")
print(f"Physical: {joystick.phys}")
print(f"Unique ID: {joystick.uniq}")
print()

# ABS_HAT0Y forward/backward
# ABS_HAT0X left/right

lastEvent = None


count = 35
Z = 60

FrontLeftArm.set_coord(45, -40 - count, Z)
FrontRightArm.set_coord(45, -40 - count, Z)
BackLeftArm.set_coord(45, -40 - count, Z)
BackRightArm.set_coord(45, -40 - count, Z)


def joystick_loop():
    global lastEvent
    roll = 0
    pitch = 0
    yaw = 0
    for event in joystick.read_loop():
        # Buttons
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)

            if key_event.keystate == key_event.key_down:
                print(f"BUTTON {event.code}: PRESSED")

            elif key_event.keystate == key_event.key_up:
                print(f"BUTTON {event.code}: RELEASED")

        # Analog sticks / triggers / D-pad
        elif event.type == ecodes.EV_ABS:
            axis_name = ecodes.ABS.get(event.code, f"ABS_{event.code}")
            lastEvent = axis_name
            
            if axis_name == 'ABS_X':
                roll = min(20, -event.value / 32000 * 20)
            if axis_name == 'ABS_RY':
                pitch = min(20, event.value / 32000 * 20)    
            if axis_name == 'ABS_RX':
                yaw = min(20, event.value / 32000 * 20)
            body.set_rotation(
                roll=roll,
                pitch=pitch,
                yaw=yaw,
            )
            print(
                f"AXIS {axis_name} ({event.code}): {event.value}"
            )

joystick_thread = threading.Thread(
    target=joystick_loop,
    daemon=True,
)

joystick_thread.start()
######
while True:
    print('Event', lastEvent)
    time.sleep(0.1)