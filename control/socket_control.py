"""Test robot client. Run this in a separate terminal from the voice app."""

import json
import threading
import time
import socketio
import requests


from websockets.sync.client import connect
from control.controller import RobotController
from devices.wt901 import Wt901

SERVER_URL = "ws://192.168.1.4:8765"
WEB_SERVER_URL = "http://192.168.1.6:3001"

ROBOT_ID = "CUBEBOT"

controller = RobotController()

controller.set_initial_position()
controller_thread = threading.Thread(
    target=controller.run,
    daemon=True,
)

controller_thread.start()

def connect_socket():
    login_response = requests.post(
        f"{WEB_SERVER_URL}/api/auth/login",
        json={
            "login": "admin",
            "password": "admin",
        },
        timeout=10,
    )
    login_response.raise_for_status()

    token = login_response.json()["token"]

    socket = socketio.Client(
        reconnection=True,
        reconnection_attempts=0,
    )

    @socket.event
    def connect():
        print("Connected to robot server")

    @socket.event
    def disconnect():
        print("Disconnected from robot server")

    @socket.event
    def connect_error(error):
        print("Connection failed:", error)

    socket.connect(
        WEB_SERVER_URL,
        auth={"token": token},
        transports=["websocket"],
    )

    return socket

imu = Wt901()

def get_imu_state():
    roll = imu.get_roll()
    pitch = imu.get_pitch()
    yaw = imu.get_yaw()
    temp = imu.get_temp()
    return {
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "temp": temp
    }

def update_loop():
    socket = connect_socket()
    global controller

    while True:
        arms_state = controller.body.get_arms_state()
        imu_state = get_imu_state()

        robot_state = {
            "type": "robot_state",
            "state": {
                **arms_state,
                "imu": imu_state
            },
        }
        print(robot_state)
        if socket.connected:
            socket.emit(
                "update_robot_state",
                robot_state,
            )
        time.sleep(0.01)

# update_thread = threading.Thread(
#     target=update_loop,
#     daemon=True,
# )

# update_thread.start()

# def main():
#     try:
#         with connect(SERVER_URL, proxy=None, open_timeout=5, close_timeout=1) as socket:
#             socket.send(json.dumps({"SET_ROBOT_ID": ROBOT_ID}))
#             print(f"Connected as {ROBOT_ID} to {SERVER_URL}", flush=True)
#             print("Waiting for messages. Press Ctrl+C to disconnect.", flush=True)
#             for message in socket:
#                 data = json.loads(message)
#                 command_id = data.get("commandId")
#                 if command_id:
#                     print(f"Command: {command_id}", flush=True)
#                     if command_id == 'move_forward':
#                         controller.move_forward()
#                     if command_id == 'position_up':
#                         controller.move_to_initial_position()
#                     if command_id == 'position_down':
#                         controller.set_bottom_position()
#                     if command_id == 'position_stop':
#                         controller.stop()
#                     if command_id == 'turn_left':
#                         controller.turn_left()
#                     if command_id == 'turn_right':
#                         controller.turn_right()
                    
#         print("Server closed the connection.", flush=True)
#     except KeyboardInterrupt:
#         print("\nDisconnected.", flush=True)
#     except Exception as exc:
#         raise SystemExit(f"Connection error: {exc}") from exc


if __name__ == "__main__":
    update_loop()
