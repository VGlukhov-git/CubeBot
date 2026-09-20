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


# source .venv_rl/bin/activate

MODEL_PATH = (
    "/home/vladimir/projects/CubeBot/"
    "RL/walk/models/crawl.onnx"
)

INPUT_DEGREES = True

CONTROL_HZ = 50.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

# Important:
# keep the same speed that was used while training this checkpoint.
MAX_JOINT_SPEED_DEG = 120.0

# Emergency tilt protection.
MAX_ROLL_DEG = 15.0
MAX_PITCH_DEG = 15.0


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


def send_to_robot(names, command):
    """
    Convert MuJoCo joint angles to physical servo angles.

    Mapping is intentionally preserved exactly as in the original script.
    """

    positions_deg = dict(
        zip(
            names,
            np.rad2deg(command).tolist(),
        )
    )

    # ------------------------------------------------------------
    # DO NOT CHANGE LEG MAPPING
    # ------------------------------------------------------------

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
        90 + positions_deg["revolute_10"],
    )

    BackRightArm.set_angle(
        180 - positions_deg["revolute_3"] + delta_a,
        90 + positions_deg["revolute_6"],
        180 - positions_deg["revolute_10"],
    )


def main():
    # ============================================================
    # ONNX
    # ============================================================

    session = ort.InferenceSession(
        MODEL_PATH,
        providers=["CPUExecutionProvider"],
    )

    model_meta = session.get_modelmeta().custom_metadata_map

    if "cubebot" not in model_meta:
        raise RuntimeError(
            "ONNX model does not contain 'cubebot' metadata. "
            "The crawl ONNX exporter must save controller metadata."
        )

    metadata = json.loads(
        model_meta["cubebot"]
    )

    hardware = metadata["hardware"]
    names = metadata["joint_names"]

    # ============================================================
    # Controller configuration
    # ============================================================

    dt = float(
        hardware.get(
            "ctrl_dt",
            CONTROL_PERIOD,
        )
    )

    # For the first robot test force the speed used during training.
    max_speed = np.deg2rad(
        MAX_JOINT_SPEED_DEG
    )

    max_acceleration = float(
        hardware["max_acceleration"]
    )

    low = np.asarray(
        hardware["joint_low"],
        dtype=np.float32,
    )

    high = np.asarray(
        hardware["joint_high"],
        dtype=np.float32,
    )

    # ------------------------------------------------------------
    # Crawl controller parameters
    # ------------------------------------------------------------

    controller = metadata.get(
        "controller",
        {}
    )

    if "default_pose" in controller:
        default_pose = np.asarray(
            controller["default_pose"],
            dtype=np.float32,
        )
    elif "default_pose" in metadata:
        default_pose = np.asarray(
            metadata["default_pose"],
            dtype=np.float32,
        )
    else:
        raise RuntimeError(
            "default_pose is missing from ONNX metadata"
        )

    if "action_scale" in controller:
        action_scale = np.asarray(
            controller["action_scale"],
            dtype=np.float32,
        )
    elif "action_scale" in metadata:
        action_scale = np.asarray(
            metadata["action_scale"],
            dtype=np.float32,
        )
    else:
        raise RuntimeError(
            "action_scale is missing from ONNX metadata"
        )

    # Scalar action_scale is also valid.
    if action_scale.ndim == 0:
        action_scale = np.full(
            12,
            float(action_scale),
            dtype=np.float32,
        )

    gait_cycle_seconds = float(
        controller.get(
            "gait_cycle_seconds",
            metadata.get(
                "gait_cycle_seconds",
                2.0,
            ),
        )
    )

    # ============================================================
    # Initial joint command
    # ============================================================

    if "initial_command" in metadata:
        command = np.asarray(
            metadata["initial_command"],
            dtype=np.float32,
        )
    else:
        command = default_pose.copy()

    velocity = np.zeros(
        12,
        dtype=np.float32,
    )

    # ============================================================
    # Preparation
    # ============================================================

    preparation = hardware.get(
        "preparation_commands",
        None,
    )

    if preparation is not None:
        preparation = np.asarray(
            preparation,
            dtype=np.float32,
        )

        if preparation.ndim != 2:
            preparation = None

    preparation_frame = 0

    # ============================================================
    # IMU initialization
    # ============================================================

    roll, pitch, yaw, temp = get_angles()

    tilt = np.asarray(
        [roll, pitch],
        dtype=np.float32,
    )

    if INPUT_DEGREES:
        tilt = np.deg2rad(tilt)

    previous_tilt = tilt.copy()

    # Gait phase starts AFTER preparation.
    gait_start_time = None

    next_cycle = time.perf_counter()

    print("======================================")
    print("CubeBot crawl controller")
    print("======================================")
    print(f"Model:              {MODEL_PATH}")
    print(f"Control dt:         {dt:.4f} s")
    print(f"Control Hz:         {1.0 / dt:.1f}")
    print(f"Gait cycle:         {gait_cycle_seconds:.3f} s")
    print(f"Max joint speed:    {MAX_JOINT_SPEED_DEG:.1f} deg/s")
    print(
        f"Max acceleration:   "
        f"{np.rad2deg(max_acceleration):.1f} deg/s^2"
    )
    print(f"Default pose:       {np.rad2deg(default_pose)}")
    print(f"Action scale:       {np.rad2deg(action_scale)} deg")
    print("======================================")

    try:
        while True:
            next_cycle += CONTROL_PERIOD

            # ====================================================
            # IMU
            # ====================================================

            roll, pitch, yaw, temp = get_angles()

            tilt = np.asarray(
                [roll, pitch],
                dtype=np.float32,
            )

            if (
                tilt.shape != (2,)
                or not np.isfinite(tilt).all()
            ):
                raise ValueError(
                    "Input must contain finite roll and pitch"
                )

            if INPUT_DEGREES:
                tilt = np.deg2rad(tilt)

            # ====================================================
            # Emergency tilt check
            # ====================================================

            roll_deg = float(
                np.rad2deg(tilt[0])
            )

            pitch_deg = float(
                np.rad2deg(tilt[1])
            )

            if (
                abs(roll_deg) > MAX_ROLL_DEG
                or abs(pitch_deg) > MAX_PITCH_DEG
            ):
                print(
                    "\nEMERGENCY STOP: "
                    f"roll={roll_deg:.1f}°, "
                    f"pitch={pitch_deg:.1f}°"
                )

                break

            # ====================================================
            # Preparation stage
            # ====================================================

            preparing = (
                preparation is not None
                and preparation_frame < len(preparation)
            )

            if preparing:
                desired = preparation[
                    preparation_frame
                ]

                preparation_frame += 1

            # ====================================================
            # Neural-network controller
            # ====================================================

            else:
                if gait_start_time is None:
                    gait_start_time = time.perf_counter()

                    # Phase starts with the current IMU state.
                    previous_tilt = tilt.copy()

                    print("\nCrawl policy started")

                # ------------------------------------------------
                # Gait phase
                # ------------------------------------------------

                elapsed = (
                    time.perf_counter()
                    - gait_start_time
                )

                phase = (
                    elapsed
                    / gait_cycle_seconds
                ) % 1.0

                phase_angle = (
                    2.0
                    * np.pi
                    * phase
                )

                phase_obs = np.asarray(
                    [
                        np.sin(phase_angle),
                        np.cos(phase_angle),
                    ],
                    dtype=np.float32,
                )

                # ------------------------------------------------
                # Observation = 18
                #
                # previous roll/pitch  = 2
                # current roll/pitch   = 2
                # commanded joints     = 12
                # phase sin/cos        = 2
                # ------------------------------------------------

                observation = np.concatenate(
                    (
                        previous_tilt / np.pi,
                        tilt / np.pi,
                        command / np.pi,
                        phase_obs,
                    )
                )

                if observation.shape != (18,):
                    raise RuntimeError(
                        "Invalid observation shape: "
                        f"{observation.shape}"
                    )

                observation = (
                    observation[None]
                    .astype(np.float32)
                )

                # ------------------------------------------------
                # ONNX inference
                # ------------------------------------------------

                action = session.run(
                    ["action"],
                    {
                        "observation": observation,
                    },
                )[0][0]

                action = np.asarray(
                    action,
                    dtype=np.float32,
                )

                if action.shape != (12,):
                    raise RuntimeError(
                        "Invalid action shape: "
                        f"{action.shape}"
                    )

                if not np.isfinite(action).all():
                    raise RuntimeError(
                        "Policy returned non-finite action"
                    )

                action = np.clip(
                    action,
                    -1.0,
                    1.0,
                )

                # ------------------------------------------------
                # IMPORTANT:
                #
                # action is NOT desired velocity.
                #
                # This is the same interpretation as during
                # MuJoCo training:
                #
                # desired =
                #     default_pose +
                #     action_scale * action
                # ------------------------------------------------

                desired = (
                    default_pose
                    + action_scale * action
                )

                desired = np.clip(
                    desired,
                    low,
                    high,
                )

            # ====================================================
            # Joint speed controller
            # ====================================================

            desired_velocity = (
                desired - command
            ) / dt

            desired_velocity = np.clip(
                desired_velocity,
                -max_speed,
                max_speed,
            )

            # ====================================================
            # Braking near joint limits
            # ====================================================

            distance_to_limit = np.maximum(
                np.where(
                    desired_velocity >= 0,
                    high - command,
                    command - low,
                ),
                0.0,
            )

            braking = np.sqrt(
                2.0
                * max_acceleration
                * distance_to_limit
            )

            desired_velocity = np.clip(
                desired_velocity,
                -braking,
                braking,
            )

            # ====================================================
            # Acceleration limit
            # ====================================================

            max_velocity_change = (
                max_acceleration
                * dt
            )

            velocity += np.clip(
                desired_velocity - velocity,
                -max_velocity_change,
                max_velocity_change,
            )

            # ====================================================
            # Integrate commanded joint position
            # ====================================================

            command = np.clip(
                command
                + velocity * dt,
                low,
                high,
            )

            # ====================================================
            # Send to physical robot
            # ====================================================

            send_to_robot(
                names,
                command,
            )

            # Current observation becomes previous observation.
            previous_tilt = tilt.copy()

            # ====================================================
            # Debug
            # ====================================================

            if gait_start_time is not None:
                print(
                    f"\r"
                    f"phase={phase:.3f} "
                    f"roll={roll_deg:+5.1f}° "
                    f"pitch={pitch_deg:+5.1f}° "
                    f"action=["
                    f"{action.min():+.2f},"
                    f"{action.max():+.2f}"
                    f"]",
                    end="",
                    flush=True,
                )

            # ====================================================
            # Maintain 50 Hz
            # ====================================================

            sleep_time = (
                next_cycle
                - time.perf_counter()
            )

            if sleep_time > 0:
                time.sleep(
                    sleep_time
                )

            else:
                print(
                    f"\nWARNING: control loop is late by "
                    f"{-sleep_time * 1000:.2f} ms"
                )

                next_cycle = (
                    time.perf_counter()
                )

    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()