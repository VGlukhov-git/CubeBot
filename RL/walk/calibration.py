#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


# ============================================================
# Project imports
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from config.robot_setup import (
    FrontLeftArm,
    FrontRightArm,
    BackLeftArm,
    BackRightArm,
    delta_a,
)


DEFAULT_MODEL_PATH = (
    "/home/vladimir/projects/CubeBot/"
    "RL/walk/models/crawl.onnx"
)


# ============================================================
# Real robot mapping
# Copied from your current working default-pose script.
# ============================================================

def send_to_robot(names, command_rad):
    positions_deg = dict(
        zip(
            names,
            np.rad2deg(command_rad).tolist(),
        )
    )

    print()
    print("Logical MuJoCo / ONNX angles:")
    print("-" * 56)

    for name in names:
        print(
            f"{name:14s}: "
            f"{positions_deg[name]:+9.3f} deg"
        )

    # --------------------------------------------------------
    # ORIGINAL REAL-ROBOT LEG MAPPING — unchanged
    # --------------------------------------------------------

    fl = (
        - positions_deg["revolute_2"], ### synced
        90 - positions_deg["revolute_11"], ### synced
        180 + positions_deg["revolute_9"], ### synced
    )

    fr = (
        - positions_deg["revolute_1"], ### synced
        90 + positions_deg["revolute_8"], ### synced
        180 - positions_deg["revolute_7"], ### synced
    )

    bl = (
        positions_deg["revolute_3"], ### synced
        90 + positions_deg["revolute_12"], ### synced
        180 - positions_deg["revolute_10"], ### synced
    )

    br = (
        positions_deg["revolute_4"], ### synced
        90 + positions_deg["revolute_6"], ### synced
        180 + positions_deg["revolute_5"], ### synced
    )

    print()
    print("Physical servo commands:")
    print("-" * 56)
    print(
        f"FrontLeft : "
        f"A={fl[0]:8.3f}  B={fl[1]:8.3f}  C={fl[2]:8.3f}"
    )
    print(
        f"FrontRight: "
        f"A={fr[0]:8.3f}  B={fr[1]:8.3f}  C={fr[2]:8.3f}"
    )
    print(
        f"BackLeft  : "
        f"A={bl[0]:8.3f}  B={bl[1]:8.3f}  C={bl[2]:8.3f}"
    )
    print(
        f"BackRight : "
        f"A={br[0]:8.3f}  B={br[1]:8.3f}  C={br[2]:8.3f}"
    )

    FrontLeftArm.set_angle(*fl)
    FrontRightArm.set_angle(*fr)
    BackLeftArm.set_angle(*bl)
    BackRightArm.set_angle(*br)


def load_model_metadata(model_path):
    session = ort.InferenceSession(
        str(model_path),
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

    names = list(
        metadata["joint_names"]
    )

    default_pose = np.asarray(
        metadata["controller"]["default_pose"],
        dtype=np.float64,
    )

    if len(names) != 12:
        raise RuntimeError(
            f"Expected 12 joint names, got {len(names)}"
        )

    if default_pose.shape != (12,):
        raise RuntimeError(
            f"Invalid default_pose shape: {default_pose.shape}"
        )

    return names, default_pose


def print_pose(names, pose):
    print()
    print("Current logical pose:")
    print("-" * 44)

    for i, (name, angle) in enumerate(
        zip(names, pose)
    ):
        print(
            f"{i:2d}: "
            f"{name:14s} "
            f"{np.rad2deg(angle):+9.3f} deg"
        )


def send_default(names, default_pose):
    print()
    print("=" * 70)
    print("Sending DEFAULT POSE")
    print("=" * 70)

    send_to_robot(
        names,
        default_pose,
    )


def test_joint(
    names,
    default_pose,
    joint_name,
    delta_deg,
    pause,
):
    if joint_name not in names:
        raise ValueError(
            f"Unknown joint: {joint_name}"
        )

    index = names.index(
        joint_name
    )

    default_angle_deg = np.rad2deg(
        default_pose[index]
    )

    print()
    print("=" * 70)
    print(
        f"Testing {joint_name}"
    )
    print(
        f"default = {default_angle_deg:+.3f} deg"
    )
    print(
        f"delta   = ±{delta_deg:.3f} deg"
    )
    print("=" * 70)

    sequence = [
        ("DEFAULT", 0.0),
        ("MINUS", -delta_deg),
        ("DEFAULT", 0.0),
        ("PLUS", +delta_deg),
        ("DEFAULT", 0.0),
    ]

    for label, delta in sequence:
        pose = default_pose.copy()

        pose[index] += np.deg2rad(
            delta
        )

        logical_deg = np.rad2deg(
            pose[index]
        )

        print()
        print(
            f">>> {label}: "
            f"{joint_name} = "
            f"{logical_deg:+.3f} deg"
        )

        send_to_robot(
            names,
            pose,
        )

        if pause > 0:
            time.sleep(
                pause
            )


def interactive_mode(
    names,
    default_pose,
):
    current_pose = default_pose.copy()

    print()
    print("=" * 70)
    print("INTERACTIVE ROBOT JOINT CALIBRATOR")
    print("=" * 70)

    print("""
Commands:

  set <joint> <angle_deg>
      Set ABSOLUTE logical MuJoCo angle.

      Example:
          set revolute_10 15

  add <joint> <delta_deg>
      Add angle to current logical position.

      Example:
          add revolute_10 -10

  default <joint>
      Reset one joint to ONNX default_pose.

  reset
      Reset all joints to ONNX default_pose.

  send
      Send current pose again.

  show
      Show current logical pose.

  list
      Show available joints.

  quit
""")

    send_to_robot(
        names,
        current_pose,
    )

    while True:
        try:
            command = input(
                "\nrobot-cal> "
            ).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not command:
            continue

        parts = command.split()
        op = parts[0].lower()

        try:
            if op in (
                "quit",
                "exit",
                "q",
            ):
                break

            elif op == "list":
                print_pose(
                    names,
                    current_pose,
                )
                continue

            elif op == "show":
                print_pose(
                    names,
                    current_pose,
                )
                continue

            elif op == "send":
                pass

            elif op == "reset":
                current_pose[:] = default_pose

            elif op == "default":
                if len(parts) != 2:
                    print(
                        "Usage: default <joint>"
                    )
                    continue

                name = parts[1]

                if name not in names:
                    print(
                        f"Unknown joint: {name}"
                    )
                    continue

                idx = names.index(
                    name
                )

                current_pose[idx] = (
                    default_pose[idx]
                )

            elif op == "set":
                if len(parts) != 3:
                    print(
                        "Usage: "
                        "set <joint> <angle_deg>"
                    )
                    continue

                name = parts[1]
                angle_deg = float(
                    parts[2]
                )

                if name not in names:
                    print(
                        f"Unknown joint: {name}"
                    )
                    continue

                idx = names.index(
                    name
                )

                current_pose[idx] = (
                    np.deg2rad(angle_deg)
                )

            elif op == "add":
                if len(parts) != 3:
                    print(
                        "Usage: "
                        "add <joint> <delta_deg>"
                    )
                    continue

                name = parts[1]
                delta_deg = float(
                    parts[2]
                )

                if name not in names:
                    print(
                        f"Unknown joint: {name}"
                    )
                    continue

                idx = names.index(
                    name
                )

                current_pose[idx] += (
                    np.deg2rad(delta_deg)
                )

            else:
                print(
                    f"Unknown command: {op}"
                )
                continue

            send_to_robot(
                names,
                current_pose,
            )

        except Exception as error:
            print(
                "ERROR:",
                error,
            )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--model",
        type=Path,
        default=Path(DEFAULT_MODEL_PATH),
    )

    parser.add_argument(
        "--joint",
        type=str,
        default=None,
        help=(
            "Test one logical MuJoCo joint "
            "around ONNX default_pose"
        ),
    )

    parser.add_argument(
        "--delta-deg",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--pause",
        type=float,
        default=1.5,
    )

    parser.add_argument(
        "--interactive",
        action="store_true",
        help=(
            "Interactive absolute/set/add calibration mode"
        ),
    )

    parser.add_argument(
        "--default-only",
        action="store_true",
        help=(
            "Send only ONNX default_pose"
        ),
    )

    args = parser.parse_args()

    names, default_pose = (
        load_model_metadata(
            args.model
        )
    )

    print()
    print("ONNX:")
    print(
        args.model
    )

    print_pose(
        names,
        default_pose,
    )

    if args.interactive:
        interactive_mode(
            names,
            default_pose,
        )

    elif args.joint is not None:
        test_joint(
            names,
            default_pose,
            args.joint,
            args.delta_deg,
            args.pause,
        )

    else:
        send_default(
            names,
            default_pose,
        )

        if not args.default_only:
            print()
            print(
                "Robot is holding the default pose. "
                "Press Ctrl+C to stop."
            )

            try:
                while True:
                    time.sleep(
                        1.0
                    )
            except KeyboardInterrupt:
                print(
                    "\nStopped"
                )


if __name__ == "__main__":
    main()
