import json
import time
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort

from gyro import get_angles


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from config.robot_setup import (
    A,
    B,
    C,
    arms,
    body,
    FrontLeftArm,
    FrontRightArm,
    BackLeftArm,
    BackRightArm,
    delta_a,
)


MODEL_PATH = (
    "/home/vladimir/projects/CubeBot/"
    "RL/walk/models/crawl.onnx"
)


CONTROL_HZ = 50.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ


def send_to_robot(names, command):
    positions_deg = dict(
        zip(
            names,
            np.rad2deg(command).tolist(),
        )
    )

    print("\nDefault pose:")
    for name in names:
        print(
            f"{name:14s}: "
            f"{positions_deg[name]:+8.3f} deg"
        )

    # ============================================================
    # ORIGINAL LEG MAPPING — unchanged
    # ============================================================

    FrontLeftArm.set_angle(
        180 + positions_deg["revolute_2"] + delta_a,
        90 - positions_deg["revolute_11"],
        180 + positions_deg["revolute_9"],
    )

    FrontRightArm.set_angle(
        180 + positions_deg["revolute_1"] + delta_a,
        90 + positions_deg["revolute_8"],
        180 - positions_deg["revolute_7"],
    )

    BackLeftArm.set_angle(
        180 - positions_deg["revolute_3"] + delta_a,
        90 + positions_deg["revolute_12"],
        180 - positions_deg["revolute_10"],
    )

    BackRightArm.set_angle(
        180 - positions_deg["revolute_3"] + delta_a,
        90 + positions_deg["revolute_6"],
        180 - positions_deg["revolute_10"],
    )


def main():
    session = ort.InferenceSession(
        MODEL_PATH,
        providers=["CPUExecutionProvider"],
    )

    model_meta = (
        session
        .get_modelmeta()
        .custom_metadata_map
    )

    if "cubebot" not in model_meta:
        raise RuntimeError(
            "ONNX model does not contain 'cubebot' metadata"
        )

    metadata = json.loads(
        model_meta["cubebot"]
    )

    names = metadata["joint_names"]

    controller = metadata["controller"]

    default_pose = np.asarray(
        controller["default_pose"],
        dtype=np.float32,
    )

    if default_pose.shape != (12,):
        raise RuntimeError(
            f"Invalid default_pose shape: "
            f"{default_pose.shape}"
        )

    print("Joint order:")

    for i, name in enumerate(names):
        print(
            f"{i:2d}: "
            f"{name:14s} "
            f"{np.rad2deg(default_pose[i]):+8.3f} deg"
        )

    print("\nSending DEFAULT POSE only...")

    send_to_robot(
        names,
        default_pose,
    )

    print("\nDefault pose sent.")

    # Keep program alive so servo controller/object isn't destroyed.
    try:
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()