import json
import time

import numpy as np
import onnxruntime as ort

from gyro import get_angles

import sys
from pathlib import Path

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
    delta_a
)

# source .venv_rl/bin/activate

MODEL_PATH = "/home/vladimir/projects/CubeBot/RL/standing_stabilisation/models/no-yaw.onnx"

INPUT_DEGREES = True

CONTROL_HZ = 50.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

JOINT_MAP = {
    "left_back": {
        "hip": "revolute_10",
        "middle": "revolute_12",
        "lower": "revolute_3",
    },
    "right_back": {
        "hip": "revolute_5",
        "middle": "revolute_6",
        "lower": "revolute_4",
    },
    "right_front": {
        "hip": "revolute_7",
        "middle": "revolute_8",
        "lower": "revolute_1",
    },
    "left_front": {
        "hip": "revolute_9",
        "middle": "revolute_11",
        "lower": "revolute_2",
    },
}

def main():
    session = ort.InferenceSession(
        MODEL_PATH,
        providers=["CPUExecutionProvider"],
    )

    metadata = json.loads(
        session.get_modelmeta().custom_metadata_map["cubebot"]
    )

    hardware = metadata["hardware"]
    names = metadata["joint_names"]

    dt = float(hardware["ctrl_dt"])
    max_speed = float(hardware["max_speed"])
    max_acceleration = float(hardware["max_acceleration"])

    low = np.asarray(
        hardware["joint_low"],
        dtype=np.float32,
    )

    high = np.asarray(
        hardware["joint_high"],
        dtype=np.float32,
    )

    preparation = np.asarray(
        hardware["preparation_commands"],
        dtype=np.float32,
    )

    command = np.asarray(
        metadata["initial_command"],
        dtype=np.float32,
    )

    velocity = np.zeros(12, dtype=np.float32)

    frame = 0

    next_cycle = time.perf_counter()

    try:
        while True:
            next_cycle += CONTROL_PERIOD

            roll, pitch, yaw, temp = get_angles()
            
            tilt = np.asarray(
                [roll, pitch],
                dtype=np.float32,
            )

            if tilt.shape != (2,) or not np.isfinite(tilt).all():
                raise ValueError(
                    "Input must contain finite roll and pitch"
                )

            if INPUT_DEGREES:
                tilt = np.deg2rad(tilt)

            # ====================================================
            # Preparation stage
            # ====================================================

            preparing = frame < len(preparation)

            if preparing:
                action = (
                    preparation[frame] - command
                ) / (dt * max_speed)

            # ====================================================
            # Neural-network controller
            # ====================================================

            else:
                observation = np.concatenate(
                    (
                        tilt / np.pi,
                        command / np.pi,
                    )
                )[None].astype(np.float32)

                action = session.run(
                    None,
                    {
                        "observation": observation,
                    },
                )[0][0]


            desired_velocity = (
                np.clip(action, -1, 1)
                * max_speed
            )

            braking = np.sqrt(
                2
                * max_acceleration
                * np.maximum(
                    np.where(
                        desired_velocity >= 0,
                        high - command,
                        command - low,
                    ),
                    0,
                )
            )

            desired_velocity = np.clip(
                desired_velocity,
                -braking,
                braking,
            )

            velocity += np.clip(
                desired_velocity - velocity,
                -max_acceleration * dt,
                max_acceleration * dt,
            )

            command = np.clip(
                command + velocity * dt,
                low,
                high,
            )

            frame += 1

            positions_deg = dict(
                zip(
                    names,
                    np.rad2deg(command).tolist(),
                )
            )

            # a, b, c
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
                positions_deg["revolute_3"] + delta_a,
                90 + positions_deg["revolute_12"],
                180 - positions_deg["revolute_10"], 
            )
            
            BackRightArm.set_angle(
                positions_deg["revolute_3"] + delta_a,
                90 + positions_deg["revolute_6"],
                180 - positions_deg["revolute_10"], 
            )
            print(positions_deg["revolute_6"])
            sleep_time = next_cycle - time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(
                    f"WARNING: control loop is late by "
                    f"{-sleep_time * 1000:.2f} ms"
                )

                next_cycle = time.perf_counter()

    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
