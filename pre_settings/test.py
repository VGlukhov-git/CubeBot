import serial
import threading
import time

COM_PORT = "/dev/ttyAMA0"     # <-- change this to your UART port
BAUDRATE = 115200*2

STEP = 0.1
DELAY = 0.001

SERVO_IDS = range(16)  # Servos 0-15

ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(2)


def reader():
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            print(line)
            if line:
                print("RX:", line)

        except Exception:
            break


def set_servo(servo, angle):
    ser.write(f"{servo};{angle:.2f}\n".encode())


threading.Thread(
    target=reader,
    daemon=True
).start()


angle = 0.0
direction = 1

try:
    while True:

        # Send same angle to all 16 servos
        for servo in SERVO_IDS:
            set_servo(servo, angle)

        angle += STEP * direction

        if angle >= 90:
            angle = 90
            direction = -1

        elif angle <= 0:
            angle = 0
            direction = 1

        time.sleep(DELAY)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    ser.close()