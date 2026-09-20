import socket
import struct
import time
import traceback

import cv2
import numpy as np

from picamera2 import Picamera2
from libcamera import controls

from depth_worker import DepthWorker


# ==========================================================
# CONFIG
# ==========================================================

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

DEPTH_WIDTH = 320
DEPTH_HEIGHT = 256

# Robot controller on the same Raspberry Pi
ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 5005

# Unity PC
UNITY_IP = "192.168.1.6"
UNITY_PORT = 5005


# ==========================================================
# UDP
# ==========================================================

depth_frame_id = 0


def send_depth_grid_udp(
    udp_socket,
    depth_grid,
):
    global depth_frame_id

    if depth_grid is None:
        return

    grid = np.ascontiguousarray(
        depth_grid,
        dtype=np.float32,
    )

    rows, cols = grid.shape

    # Header:
    # uint16 rows
    # uint16 cols
    # uint32 frame_id
    header = struct.pack(
        "<HHI",
        rows,
        cols,
        depth_frame_id,
    )

    packet = (
        header
        + grid.tobytes()
    )

    # ------------------------------------------------------
    # Robot controller
    # ------------------------------------------------------

    udp_socket.sendto(
        packet,
        (
            ROBOT_IP,
            ROBOT_PORT,
        ),
    )

    # ------------------------------------------------------
    # Unity
    # ------------------------------------------------------

    udp_socket.sendto(
        packet,
        (
            UNITY_IP,
            UNITY_PORT,
        ),
    )

    depth_frame_id = (
        depth_frame_id + 1
    ) & 0xFFFFFFFF


# ==========================================================
# CAMERA LOOP
# ==========================================================

def camera_loop():

    print("Starting AI camera...")

    # ------------------------------------------------------
    # UDP
    # ------------------------------------------------------

    udp_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    # ------------------------------------------------------
    # Camera
    # ------------------------------------------------------

    camera = Picamera2()

    camera_config = (
        camera.create_video_configuration(
            main={
                "size": (
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                ),
                "format": "RGB888",
            },

            lores={
                "size": (
                    DEPTH_WIDTH,
                    DEPTH_HEIGHT,
                ),
                "format": "RGB888",
            },
        )
    )

    camera.configure(
        camera_config
    )

    print(
        "Camera configuration:",
        camera.camera_configuration(),
    )

    camera.start()

    camera.set_controls({
        "AfMode":
            controls.AfModeEnum.Continuous
    })

    time.sleep(1)

    print("Camera started")

    # ------------------------------------------------------
    # Depth worker
    # ------------------------------------------------------

    depth = DepthWorker()

    depth.start()

    print("Depth worker started")

    # ------------------------------------------------------
    # Validate Hailo size
    # ------------------------------------------------------

    print(
        f"Camera depth stream: "
        f"{DEPTH_WIDTH}x{DEPTH_HEIGHT}"
    )

    print(
        f"Hailo input: "
        f"{depth.input_w}x{depth.input_h}"
    )

    if (
        depth.input_w != DEPTH_WIDTH
        or
        depth.input_h != DEPTH_HEIGHT
    ):
        raise RuntimeError(
            "Camera lores stream does not match "
            "Hailo model input: "
            f"camera={DEPTH_WIDTH}x{DEPTH_HEIGHT}, "
            f"Hailo={depth.input_w}x{depth.input_h}"
        )

    # ------------------------------------------------------
    # Runtime state
    # ------------------------------------------------------

    last_depth_id = -1

    camera_fps_avg = 0.0
    last_print = 0.0

    try:

        while True:

            t0 = time.perf_counter()

            # --------------------------------------------------
            # Capture LOW RESOLUTION stream only
            # --------------------------------------------------

            frame = camera.capture_array(
                "lores"
            )

            if frame is None:
                continue

            # --------------------------------------------------
            # Rotate
            # --------------------------------------------------

            frame = cv2.rotate(
                frame,
                cv2.ROTATE_180,
            )

            # --------------------------------------------------
            # Important:
            #
            # lores is already:
            #
            # 320 x 256
            # RGB888
            #
            # So:
            #
            # NO cv2.resize()
            # NO cv2.cvtColor()
            # --------------------------------------------------

            depth.set_frame(
                frame
            )

            # --------------------------------------------------
            # Get newest completed depth result
            # --------------------------------------------------

            current_depth_id = (
                depth.get_frame_id()
            )

            if (
                current_depth_id
                != last_depth_id
            ):

                depth_grid = (
                    depth.get_depth_grid(
                        copy=False
                    )
                )

                if depth_grid is not None:

                    try:
                        send_depth_grid_udp(
                            udp_socket,
                            depth_grid,
                        )

                    except OSError as e:
                        print(
                            "Depth UDP error:",
                            repr(e),
                        )

                    last_depth_id = (
                        current_depth_id
                    )

            # --------------------------------------------------
            # Camera loop FPS
            # --------------------------------------------------

            dt = (
                time.perf_counter()
                - t0
            )

            if dt > 0:

                fps = 1.0 / dt

                camera_fps_avg = (
                    fps
                    if camera_fps_avg == 0
                    else
                    camera_fps_avg * 0.9
                    + fps * 0.1
                )

            # --------------------------------------------------
            # Debug once per second
            # --------------------------------------------------

            now = time.monotonic()

            if (
                now - last_print
                >= 1.0
            ):

                last_print = now

                print(
                    f"Camera: "
                    f"{camera_fps_avg:.1f} FPS | "
                    f"Depth: "
                    f"{depth.fps:.1f} FPS | "
                    f"Sent frame: "
                    f"{depth_frame_id}"
                )

    except KeyboardInterrupt:

        print(
            "\nStopping AI camera..."
        )

    except Exception as e:

        print(
            "AI CAMERA CRASHED:",
            repr(e),
        )

        traceback.print_exc()

    finally:

        print(
            "Cleaning up camera..."
        )

        try:
            depth.stop()
        except Exception:
            pass

        try:
            camera.stop()
        except Exception:
            pass

        try:
            udp_socket.close()
        except Exception:
            pass

        print(
            "AI camera stopped"
        )


# ==========================================================
# RUN
# ==========================================================

if __name__ == "__main__":
    camera_loop()