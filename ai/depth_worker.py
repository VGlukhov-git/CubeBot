import threading
import time

import cv2
import numpy as np

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


DEPTH_HEF_PATH = (
    "/home/vladimir/projects/CubeBot/ai/models/"
    "scdepthv3--320x256_quant_hailort_multidevice_1.hef"
)

DEPTH_GRID_COLS = 50
DEPTH_GRID_ROWS = 50
DEPTH_GRID_STAT = "median"


def extract_depth_grid(
    depth_map,
    cols,
    rows,
    statistic="center",
):
    h, w = depth_map.shape[:2]

    if statistic == "center":
        x_edges = np.linspace(
            0,
            w,
            cols + 1,
            dtype=np.int32,
        )

        y_edges = np.linspace(
            0,
            h,
            rows + 1,
            dtype=np.int32,
        )

        x_centers = (
            x_edges[:-1] + x_edges[1:]
        ) // 2

        y_centers = (
            y_edges[:-1] + y_edges[1:]
        ) // 2

        x_centers = np.clip(
            x_centers,
            0,
            w - 1,
        )

        y_centers = np.clip(
            y_centers,
            0,
            h - 1,
        )

        return depth_map[
            y_centers[:, None],
            x_centers[None, :],
        ].astype(
            np.float32,
            copy=False,
        )

    # --------------------------------------------------
    # Fast path for mean / median
    #
    # Resize/crop so the image is evenly divisible
    # into rows x cols cells.
    # --------------------------------------------------

    cell_h = h // rows
    cell_w = w // cols

    usable_h = cell_h * rows
    usable_w = cell_w * cols

    if cell_h == 0 or cell_w == 0:
        raise ValueError(
            "Grid resolution is larger than depth map"
        )

    cropped = depth_map[
        :usable_h,
        :usable_w,
    ]

    # Shape:
    #
    # (rows, cell_h, cols, cell_w)
    #
    cells = cropped.reshape(
        rows,
        cell_h,
        cols,
        cell_w,
    )

    if statistic == "mean":
        values = np.nanmean(
            cells,
            axis=(1, 3),
        )

    elif statistic == "median":
        values = np.nanmedian(
            cells,
            axis=(1, 3),
        )

    else:
        raise ValueError(
            'statistic must be '
            '"center", "mean", or "median"'
        )

    return values.astype(
        np.float32,
        copy=False,
    )


class DepthWorker:
    def __init__(
        self,
        hef_path=DEPTH_HEF_PATH,
        grid_cols=DEPTH_GRID_COLS,
        grid_rows=DEPTH_GRID_ROWS,
        statistic=DEPTH_GRID_STAT,
    ):
        self.hef_path = hef_path

        self.grid_cols = grid_cols
        self.grid_rows = grid_rows
        self.statistic = statistic

        self.running = False
        self.thread = None

        self.frame_lock = threading.Lock()
        self.latest_frame = None

        self.depth_lock = threading.Lock()

        self.latest_depth_map = None
        self.latest_depth_grid = None

        self.frame_id = 0
        self.fps = 0.0

        #
        # Initialize Hailo
        #
        self.device = VDevice()

        self.hef = HEF(self.hef_path)

        config = ConfigureParams.create_from_hef(
            self.hef,
            interface=HailoStreamInterface.PCIe,
        )

        self.network_group = self.device.configure(
            self.hef,
            config,
        )[0]

        self.input_params = (
            InputVStreamParams.make_from_network_group(
                self.network_group,
                quantized=True,
                format_type=FormatType.UINT8,
            )
        )

        self.output_params = (
            OutputVStreamParams.make_from_network_group(
                self.network_group,
                quantized=False,
                format_type=FormatType.FLOAT32,
            )
        )

        self.pipeline = InferVStreams(
            self.network_group,
            self.input_params,
            self.output_params,
        )

        self.pipeline.__enter__()

        self.input_info = (
            self.hef.get_input_vstream_infos()[0]
        )

        self.output_info = (
            self.hef.get_output_vstream_infos()[0]
        )

        self.input_h, self.input_w, _ = (
            self.input_info.shape
        )

        self.network_params = (
            self.network_group.create_params()
        )

        print(
            f"Depth model input: "
            f"{self.input_w}x{self.input_h}"
        )

    # ------------------------------------------------
    # Input
    # ------------------------------------------------

    def set_frame(self, frame):
        """
        Give the depth worker a new camera frame.

        The old unprocessed frame is discarded.
        """

        with self.frame_lock:
            self.latest_frame = frame

    # ------------------------------------------------
    # Output
    # ------------------------------------------------

    def get_depth_grid(self, copy=True):
        with self.depth_lock:
            if self.latest_depth_grid is None:
                return None

            if copy:
                return self.latest_depth_grid.copy()

            return self.latest_depth_grid

    def get_depth_map(self, copy=True):
        with self.depth_lock:
            if self.latest_depth_map is None:
                return None

            if copy:
                return self.latest_depth_map.copy()

            return self.latest_depth_map

    def get_frame_id(self):
        with self.depth_lock:
            return self.frame_id

    # ------------------------------------------------
    # Thread
    # ------------------------------------------------

    def start(self):
        if self.running:
            return

        self.running = True

        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="depth-inference",
        )

        self.thread.start()

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=2)

        try:
            self.pipeline.__exit__(
                None,
                None,
                None,
            )
        except Exception:
            pass

        try:
            self.device.release()
        except Exception:
            pass

    # ------------------------------------------------
    # Inference loop
    # ------------------------------------------------

    def _run(self):
        fps_avg = 0.0

        while self.running:
            #
            # Grab latest frame
            #
            with self.frame_lock:
                frame = self.latest_frame
                self.latest_frame = None

            #
            # Nothing new yet
            #
            if frame is None:
                time.sleep(0.001)
                continue

            t0 = time.perf_counter()
            #
            # Picamera2 RGB888 note:
            #
            # If frame is already RGB, you DON'T
            # need BGR -> RGB conversion.
            #
            depth_input = cv2.resize(
                frame,
                (self.input_w, self.input_h),
                interpolation=cv2.INTER_LINEAR,
            )

            depth_input = np.expand_dims(
                depth_input,
                axis=0,
            )

            with self.network_group.activate(
                self.network_params
            ):
                depth_out = self.pipeline.infer({
                    self.input_info.name: depth_input,
                })

            raw_depth = depth_out[
                self.output_info.name
            ][0]

            raw_depth = np.squeeze(raw_depth)

            depth_grid = extract_depth_grid(
                raw_depth,
                self.grid_cols,
                self.grid_rows,
                self.statistic,
            )

            #
            # Store results atomically
            #
            
            with self.depth_lock:
                self.latest_depth_map = raw_depth
                self.latest_depth_grid = depth_grid
                self.frame_id += 1

            dt = time.perf_counter() - t0

            if dt > 0:
                fps = 1.0 / dt

                fps_avg = (
                    fps
                    if fps_avg == 0
                    else fps_avg * 0.9 + fps * 0.1
                )

                self.fps = fps_avg