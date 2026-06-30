import serial
import time

COM_PORT = "/dev/ttyAMA0"     # <-- change this to your UART port
BAUDRATE = 115200*2

NUM_SERVOS = 16
STEP = 2             # degrees per step
DELAY = 0.11        # seconds between updates

ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(1)         # allow Pico to reset

def set_servo(servo, angle):
    print(angle)
    ser.write(f"{servo};{angle/10.0:.2f}\n".encode())

# All to 90
# for s in range(NUM_SERVOS):
#         set_servo(s, 1650/2)
#         print(f"Servo {s} set to 90.0°")
#         time.sleep(0.1)

servos_0 = [1, 5, 12, 14] ## C
servos_1 = [2, 3, 11, 13] ## A
servos_2 = [0, 4, 10, 15] ## B

deltas_0 =[0, 0, 50, 50] #fr fl bl br
deltas_1 =[-30, -40, 0, 0] #fr fl bl br
deltas_2 =[0, 100, 0, 30] #fr fl bl br


# for i, s in enumerate(servos_1):
#     set_servo(s, 1800 / 2 +  + deltas_1[i])
#     print(f"Index {i}, Servo {s} set to 90.0°")
#     time.sleep(1)

for i, s in enumerate(servos_2):
    set_servo(s, 1800 / 2 +  + deltas_0[i])
    print(f"Index {i}, Servo {s} set to 90.0°")
    time.sleep(1)
## front right  = 1, 0, 2 //
## front left = 5, 4, 3 // 4 +100
## back left = 10, 11, 12 //
## back right = 13, 14 ,15 //