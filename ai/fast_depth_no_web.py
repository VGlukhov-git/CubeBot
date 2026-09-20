import atexit
import socket
import struct
import time

import cv2
import numpy as np

from picamera2 import Picamera2
from libcamera import controls

from hailo_platform import (
    HEF,
    VDevice,
    ConfigureParams,
    HailoStreamInterface,
    InputVStreamParams,
    OutputVStreamParams,
    InferVStreams,
    FormatType,
)

DEPTH_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/scdepthv3--320x256_quant_hailort_multidevice_1.hef"

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

DEPTH_GRID_COLS = 50
DEPTH_GRID_ROWS = 50
DEPTH_GRID_STAT = "median"  # "center" | "mean" | "median"

UNITY_IP = "192.168.1.6"
UNITY_PORT = 5005

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
depth_frame_id = 0


def send_depth_grid_udp(depth_grid):
    global depth_frame_id

    grid = np.ascontiguousarray(depth_grid, dtype=np.float32)
    rows, cols = grid.shape

    # uint16 rows, uint16 cols, uint32 frame_id
    header = struct.pack("<HHI", rows, cols, depth_frame_id)
    packet = header + grid.tobytes()

    udp_socket.sendto(packet, (UNITY_IP, UNITY_PORT))
    depth_frame_id = (depth_frame_id + 1) & 0xFFFFFFFF


def extract_depth_grid(depth_map, cols, rows, statistic="center"):
    h, w = depth_map.shape[:2]

    values = np.full((rows, cols), np.nan, dtype=np.float32)

    x_edges = np.linspace(0, w, cols + 1, dtype=np.int32)
    y_edges = np.linspace(0, h, rows + 1, dtype=np.int32)

    for row in range(rows):
        y1, y2 = y_edges[row], y_edges[row + 1]

        for col in range(cols):
            x1, x2 = x_edges[col], x_edges[col + 1]

            if statistic == "center":
                cx = min(w - 1, (x1 + x2) // 2)
                cy = min(h - 1, (y1 + y2) // 2)

                value = depth_map[cy, cx]

                if np.isfinite(value):
                    values[row, col] = float(value)

                continue

            cell = depth_map[y1:y2, x1:x2]
            valid = cell[np.isfinite(cell)]

            if valid.size == 0:
                continue

            if statistic == "mean":
                value = np.mean(valid)
            elif statistic == "median":
                value = np.median(valid)
            else:
                raise ValueError(
                    'DEPTH_GRID_STAT must be "center", "mean", or "median"'
                )

            values[row, col] = float(value)

    return values


camera = Picamera2()
camera.configure(
    camera.create_video_configuration(
        main={
            "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
            "format": "RGB888",
        }
    )
)

camera.start()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
time.sleep(1)

device = VDevice()


def setup_depth_model():
    hef = HEF(DEPTH_HEF_PATH)

    config = ConfigureParams.create_from_hef(
        hef,
        interface=HailoStreamInterface.PCIe,
    )

    network_group = device.configure(hef, config)[0]

    input_params = InputVStreamParams.make_from_network_group(
        network_group,
        quantized=True,
        format_type=FormatType.UINT8,
    )

    output_params = OutputVStreamParams.make_from_network_group(
        network_group,
        quantized=False,
        format_type=FormatType.FLOAT32,
    )

    pipeline = InferVStreams(
        network_group,
        input_params,
        output_params,
    )

    return hef, network_group, pipeline


depth_hef, depth_ng, depth_pipeline = setup_depth_model()
depth_pipeline.__enter__()

depth_input_info = depth_hef.get_input_vstream_infos()[0]
depth_output_info = depth_hef.get_output_vstream_infos()[0]

DEPTH_H, DEPTH_W, _ = depth_input_info.shape
depth_params = depth_ng.create_params()

print(f"Depth input: {DEPTH_W}x{DEPTH_H}")
print(
    f"Sending {DEPTH_GRID_COLS}x{DEPTH_GRID_ROWS} depth grid "
    f"to {UNITY_IP}:{UNITY_PORT}"
)


def cleanup():
    try:
        camera.stop()
    except Exception:
        pass

    try:
        depth_pipeline.__exit__(None, None, None)
    except Exception:
        pass

    try:
        device.release()
    except Exception:
        pass

    try:
        udp_socket.close()
    except Exception:
        pass


atexit.register(cleanup)


def main():
    fps_avg = 0.0

    while True:
        t0 = time.perf_counter()

        frame = camera.capture_array()

        if frame is None:
            continue

        frame = cv2.rotate(frame, cv2.ROTATE_180)

        depth_input = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        depth_input = cv2.resize(
            depth_input,
            (DEPTH_W, DEPTH_H),
            interpolation=cv2.INTER_LINEAR,
        )
        depth_input = np.expand_dims(depth_input, axis=0)

        with depth_ng.activate(depth_params):
            depth_out = depth_pipeline.infer({
                depth_input_info.name: depth_input,
            })

        raw_depth = depth_out[depth_output_info.name][0]
        raw_depth = np.squeeze(raw_depth)

        depth_grid = extract_depth_grid(
            raw_depth,
            DEPTH_GRID_COLS,
            DEPTH_GRID_ROWS,
            DEPTH_GRID_STAT,
        )

        try:
            send_depth_grid_udp(depth_grid)
        except OSError as e:
            print("UDP send error:", e)

        dt = time.perf_counter() - t0

        if dt > 0:
            fps = 1.0 / dt
            fps_avg = fps if fps_avg == 0 else fps_avg * 0.9 + fps * 0.1

        if depth_frame_id % 100 == 0:
            print(
                f"FPS: {fps_avg:.1f} | "
                f"frame: {depth_frame_id} | "
                f"grid: {depth_grid.shape}"
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
