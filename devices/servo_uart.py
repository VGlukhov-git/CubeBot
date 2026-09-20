import serial as pyserial
import time
import threading

COM_PORT = "/dev/ttyAMA0"     # <-- change this
BAUDRATE = 115200*2

serial = pyserial.Serial(
    COM_PORT,
    BAUDRATE,
    timeout=2,
    write_timeout=1
)
time.sleep(1)         # allow Pico to reset

def set_servo(servo, angle):
    serial.write(f"{servo};{angle/10.0:.1f}\n".encode())

def uart_reader():
    while True:
        try:
            line = serial.readline().decode("utf-8").strip()
            if line:
                print("RX:", line)
        except Exception as e:
            print("RX ERROR:", e)
MIN = 0
MAX = 1800

delta = (MAX - MIN) / 180.0

# threading.Thread(target=uart_reader, daemon=True).start()

class ServoUart:
    def __init__(self):
        print("Initializing servo UART")

    def send_message(self, channel, angle):
        value = round(delta * angle, 2)
        set_servo(channel, value)
        # print("SEND:", channel, angle)
