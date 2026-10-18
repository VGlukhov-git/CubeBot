import time

from points.animation import get_walk_points
from config.robot_setup import (
    body,
    FrontLeftArm,
    FrontRightArm,
    BackLeftArm,
    BackRightArm,
)


class RobotController:
    def __init__(self):
        self.body = body

        self.front_left = FrontLeftArm
        self.front_right = FrontRightArm
        self.back_left = BackLeftArm
        self.back_right = BackRightArm
        self.Z = 60
        self.set_count = 50

        self.rotate_direction = 1

        self.yaw = 35
        self.d_yaw = self.yaw / 3

        self.dX = 10
        self.dZ = 10
        self.dY = 30

        self.phase = 0
        self.local_yaw = 0.0

        self.delta = 1
        self.count = 20

        self.local_d_yaw = self.d_yaw / self.count
        self.local_dy = self.dY / self.count

        self.last_dir = self.rotate_direction

        self.should_prepare_for_rotation = True
        self.can_rotate = False
        self.finished = True

        self.can_move = False

        self.current_points = [0, 0, 0, 0]

        self.K = 1

        self.walk_count = 25 * self.K
        self.walk_count_up = 11 * self.K

        self.walk_points = get_walk_points(
            self.walk_count,
            self.walk_count_up,
            0
        )

        self.running = False

    def all_legs_next(self):
        return [
            self.front_left.next(),
            self.front_right.next(),
            self.back_left.next(),
            self.back_right.next(),
        ]

    def wait_until_all_legs_ready(self):
        is_all_legs_ready = False
        self.isRunning = True
        while not is_all_legs_ready:
            is_all_legs_ready = all([
                self.front_left.next(),
                self.front_right.next(),
                self.back_left.next(),
                self.back_right.next(),
            ])

            time.sleep(0.007)
        self.isRunning = False

    def prepare_for_movement(self):
        self.front_left.set_position(
            self.walk_points["front_left"][0]["x"],
            self.walk_points["front_left"][0]["y"],
            self.walk_points["front_left"][0]["z"],
            self.walk_points["front_left"][0]["count"],
        )

        self.front_right.set_position(
            self.walk_points["front_right"][0]["x"],
            self.walk_points["front_right"][0]["y"],
            self.walk_points["front_right"][0]["z"],
            self.walk_points["front_right"][0]["count"],
        )

        self.back_left.set_position(
            self.walk_points["back_left"][0]["x"],
            self.walk_points["back_left"][0]["y"],
            self.walk_points["back_left"][0]["z"],
            self.walk_points["back_left"][0]["count"],
        )

        self.back_right.set_position(
            self.walk_points["back_right"][0]["x"],
            self.walk_points["back_right"][0]["y"],
            self.walk_points["back_right"][0]["z"],
            self.walk_points["back_right"][0]["count"],
        )

    def set_initial_position(self):
        y = -40 - 35

        self.front_left.set_position(
            45,
            y,
            self.Z,
            self.set_count,
        )

        self.front_right.set_position(
            45,
            y,
            self.Z,
            self.set_count,
        )

        self.back_left.set_position(
            45,
            y,
            self.Z,
            self.set_count,
        )

        self.back_right.set_position(
            45,
            y,
            self.Z,
            self.set_count,
        )

        self.wait_until_all_legs_ready()

    def set_bottom_position(self):
        z = 20
        self.front_left.set_position(
            0,
            -40,
            z,
            self.set_count,
        )

        self.front_right.set_position(
            0,
            -40,
            z,
            self.set_count,
        )

        self.back_left.set_position(
            0,
            -40,
            z,
            self.set_count,
        )

        self.back_right.set_position(
            0,
            -40,
            z,
            self.set_count,
        )

        self.wait_until_all_legs_ready()

    def prepare_for_rotation(self):
        # Front left
        self.body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=-self.d_yaw * 1.5,
            exclude_legs=[
                "front_right",
                "back_left",
                "back_right",
            ],
        )

        # Front right
        self.body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=self.d_yaw * 1.5,
            exclude_legs=[
                "front_left",
                "back_left",
                "back_right",
            ],
        )

        # Back left
        self.body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=self.d_yaw * -0.5,
            exclude_legs=[
                "front_left",
                "front_right",
                "back_right",
            ],
        )

        # Back right
        self.body.set_rotation_local(
            roll=0,
            pitch=0,
            yaw=self.d_yaw * 0.5,
            exclude_legs=[
                "front_left",
                "front_right",
                "back_left",
            ],
        )

    def move_to_initial_position(self):
        self.can_rotate = False
        self.finished = True

        self.phase = 0
        self.local_yaw = 0.0

        self.last_dir = self.rotate_direction

        self.should_prepare_for_rotation = True

        self.body.reset_state()

        self.set_initial_position()


    def process_movement(self):
        if not self.can_move:
            return

        legs_ready = [
            self.front_left.next(),
            self.front_right.next(),
            self.back_left.next(),
            self.back_right.next(),
        ]

        # --------------------------------------------------------
        # Front left
        # --------------------------------------------------------
        if legs_ready[0]:
            self.current_points[0] += 1

            if self.current_points[0] == len(
                self.walk_points["front_left"]
            ):
                self.current_points[0] = 0

            point = self.walk_points[
                "front_left"
            ][self.current_points[0]]

            self.front_left.set_position(
                point["x"],
                point["y"],
                point["z"],
                point["count"],
            )

        # --------------------------------------------------------
        # Front right
        # --------------------------------------------------------
        if legs_ready[1]:
            self.current_points[1] += 1

            if self.current_points[1] == len(
                self.walk_points["front_right"]
            ):
                self.current_points[1] = 0

            point = self.walk_points[
                "front_right"
            ][self.current_points[1]]

            self.front_right.set_position(
                point["x"],
                point["y"],
                point["z"],
                point["count"],
            )

        # --------------------------------------------------------
        # Back left
        # --------------------------------------------------------
        if legs_ready[2]:
            self.current_points[2] += 1

            if self.current_points[2] == len(
                self.walk_points["back_left"]
            ):
                self.current_points[2] = 0

            point = self.walk_points[
                "back_left"
            ][self.current_points[2]]

            self.back_left.set_position(
                point["x"],
                point["y"],
                point["z"],
                point["count"],
            )

        # --------------------------------------------------------
        # Back right
        # --------------------------------------------------------
        if legs_ready[3]:
            self.current_points[3] += 1

            if self.current_points[3] == len(
                self.walk_points["back_right"]
            ):
                self.current_points[3] = 0

            point = self.walk_points[
                "back_right"
            ][self.current_points[3]]

            self.back_right.set_position(
                point["x"],
                point["y"],
                point["z"],
                point["count"],
            )

    def process_rotation(self):
        if not self.can_rotate and self.finished:
            return

        self.local_d_yaw = self.d_yaw / self.count

        should_make_revert = (
            self.rotate_direction != self.last_dir
        )

        if should_make_revert:
            self.last_dir = self.rotate_direction

            self.phase += self.rotate_direction

            if self.phase > 7:
                self.phase = 0

            if self.phase < 0:
                self.phase = 7

        # --------------------------------------------------------
        # Phase 0 - Front Right
        # --------------------------------------------------------
        if self.phase == 0:
            self.body.front_right.move(
                y=self.local_dy * self.rotate_direction
            )

            # Return leg back
            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    -3
                    * self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "front_left",
                    "back_left",
                    "back_right",
                ],
            )

            # Rotate body
            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "front_right",
                ],
            )

        # --------------------------------------------------------
        # Phase 1
        # --------------------------------------------------------
        elif self.phase == 1:
            self.body.front_right.move(
                y=-self.local_dy * self.rotate_direction
            )

        # --------------------------------------------------------
        # Phase 2 - Back Right
        # --------------------------------------------------------
        elif self.phase == 2:
            self.body.back_right.move(
                y=self.local_dy * self.rotate_direction
            )

            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    -3
                    * self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "front_left",
                    "back_left",
                    "front_right",
                ],
            )

            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "back_right",
                ],
            )

        # --------------------------------------------------------
        # Phase 3
        # --------------------------------------------------------
        elif self.phase == 3:
            self.body.back_right.move(
                y=-self.local_dy * self.rotate_direction
            )

        # --------------------------------------------------------
        # Phase 4 - Back Left
        # --------------------------------------------------------
        elif self.phase == 4:
            self.body.back_left.move(
                y=self.local_dy * self.rotate_direction
            )

            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    -3
                    * self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "front_left",
                    "back_right",
                    "front_right",
                ],
            )

            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "back_left",
                ],
            )

        # --------------------------------------------------------
        # Phase 5
        # --------------------------------------------------------
        elif self.phase == 5:
            self.body.back_left.move(
                y=-self.local_dy * self.rotate_direction
            )

        # --------------------------------------------------------
        # Phase 6 - Front Left
        # --------------------------------------------------------
        elif self.phase == 6:
            self.body.front_left.move(
                y=self.local_dy * self.rotate_direction
            )

            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    -3
                    * self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "back_left",
                    "back_right",
                    "front_right",
                ],
            )

            self.body.set_rotation_local(
                roll=0,
                pitch=0,
                yaw=(
                    self.local_d_yaw
                    * self.rotate_direction
                ),
                exclude_legs=[
                    "front_left",
                ],
            )

        # --------------------------------------------------------
        # Phase 7
        # --------------------------------------------------------
        elif self.phase == 7:
            self.body.front_left.move(
                y=-self.local_dy * self.rotate_direction
            )

        # --------------------------------------------------------
        # Phase progress
        # --------------------------------------------------------
        if self.local_yaw <= self.d_yaw:
            self.local_yaw += self.local_d_yaw
            self.finished = False

        if self.local_yaw > self.d_yaw:
            self.phase += self.rotate_direction

            self.finished = True

            if self.phase > 7:
                self.phase = 0

            if self.phase < 0:
                self.phase = 7

            self.local_yaw = 0
    def stop(self):
        self.can_move = False
        self.can_rotate = False
        self.finished = True

        self.phase = 0
        self.local_yaw = 0.0
        self.last_dir = self.rotate_direction

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.should_prepare_for_rotation = True

        self.body.reset_state()
        self.set_initial_position()

    def move_forward(self):
        self.prepare_for_movement()
        self.current_points = [0,0,0,0]
        self.should_prepare_for_rotation = True
        self.can_rotate = False
        self.can_move = True
    def turn_right(self):
        self.can_rotate = True
        self.rotate_direction = 1
        if self.should_prepare_for_rotation:
            self.last_dir = self.rotate_direction
            self.should_prepare_for_rotation = False
            self.prepare_for_rotation()
    def turn_left(self):
        self.can_rotate = True
        self.rotate_direction = -1
        if self.should_prepare_for_rotation:
            self.phase = 7
            self.last_dir = self.rotate_direction
            self.should_prepare_for_rotation = False
            self.prepare_for_rotation()

    def run(self):
        self.running = True

        while self.running:
            self.process_movement()
            self.process_rotation()

            time.sleep(0.01)
