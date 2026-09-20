from evdev import InputDevice, ecodes, categorize

DEVICE = "/dev/input/event5"

joystick = InputDevice(DEVICE)

print(f"Joystick: {joystick.name}")
print(f"Path: {joystick.path}")
print(f"Physical: {joystick.phys}")
print(f"Unique ID: {joystick.uniq}")
print()

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

        print(
            f"AXIS {axis_name} ({event.code}): {event.value}"
        )