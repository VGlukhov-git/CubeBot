import math
import numpy as np


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ], dtype=float)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ], dtype=float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ], dtype=float)


class LegController:
    def __init__(self, body, leg_name):
        self.body = body
        self.leg_name = leg_name

    @property
    def arm(self):
        return self.body.legs[
            self.leg_name
        ]["arm"]

    def get_position(self):
        return np.array(
            self.arm.get_coord(),
            dtype=float,
        )

    def get_angles(self):
        return np.array(
            self.arm.get_angles(),
            dtype=float,
        )

    def get_state(self):
        return self.arm.get_state()

    def set_position(
        self,
        x=None,
        y=None,
        z=None,
    ):
        current = self.get_position()

        new_x = (
            current[0]
            if x is None
            else x
        )

        new_y = (
            current[1]
            if y is None
            else y
        )

        new_z = (
            current[2]
            if z is None
            else z
        )

        self.arm.set_coord(
            float(new_x),
            float(new_y),
            float(new_z),
        )

        return np.array([
            new_x,
            new_y,
            new_z,
        ], dtype=float)

    def move(
        self,
        x=None,
        y=None,
        z=None,
    ):
        current = self.get_position()

        delta_x = (
            0
            if x is None
            else x
        )

        delta_y = (
            0
            if y is None
            else y
        )

        delta_z = (
            0
            if z is None
            else z
        )

        new_position = (
            current
            + np.array([
                delta_x,
                delta_y,
                delta_z,
            ], dtype=float)
        )

        self.arm.set_coord(
            *new_position
        )

        return new_position


class Body:
    def __init__(self, legs):
        self.legs = legs

        self.R = np.eye(3, dtype=float)
        self.position = np.zeros(3, dtype=float)

        # Creates:
        # body.front_right
        # body.front_left
        # body.back_right
        # body.back_left
        for leg_name in self.legs:
            if hasattr(self, leg_name):
                raise ValueError(
                    f"Leg name '{leg_name}' conflicts with a Body attribute"
                )

            setattr(
                self,
                leg_name,
                LegController(self, leg_name),
            )

    def set_rotation_matrix(self, roll=0, pitch=0, yaw=0):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        self.R = (
            rot_z(yaw)
            @ rot_y(pitch)
            @ rot_x(roll)
        )

    def transform_leg_point(self, leg_name, x, y, z):
        leg = self.legs[leg_name]

        mount = np.asarray(
            leg["mount"],
            dtype=float,
        )

        local_to_body = np.asarray(
            leg["local_to_body"],
            dtype=float,
        )

        foot_world = np.array([
            x,
            z,
            y,
        ], dtype=float)

        target_body = self.R.T @ (
            foot_world - self.position
        )

        target_from_mount = (
            target_body - mount
        )

        target_leg = (
            local_to_body.T
            @ target_from_mount
        )

        return np.array([
            target_leg[0],
            target_leg[2],
            target_leg[1],
        ], dtype=float)

    def set_leg_coord(
        self,
        leg_name,
        x=None,
        y=None,
        z=None,
    ):
        """
        Set independent leg-local coordinates.

        Equivalent to:
            body.front_right.set_position(...)
        """
        return getattr(
            self,
            leg_name,
        ).set_position(
            x=x,
            y=y,
            z=z,
        )

    def move_leg(
        self,
        leg_name,
        x=None,
        y=None,
        z=None,
    ):
        """
        Increment independent leg-local coordinates.

        Equivalent to:
            body.front_right.move(...)
        """
        return getattr(
            self,
            leg_name,
        ).move(
            x=x,
            y=y,
            z=z,
        )

    def get_arms_state(self):
        return {
            name: leg["arm"].get_state()
            for name, leg in self.legs.items()
        }

    def _apply_transform(
        self,
        new_R=None,
        new_position=None,
        exclude_legs=None,
    ):
        old_R = self.R.copy()
        old_position = self.position.copy()

        if new_R is None:
            new_R = old_R

        if new_position is None:
            new_position = old_position

        new_R = np.asarray(
            new_R,
            dtype=float,
        )

        new_position = np.asarray(
            new_position,
            dtype=float,
        )

        if exclude_legs is None:
            exclude_legs = set()
        else:
            exclude_legs = set(exclude_legs)

        for name, leg in self.legs.items():
            if name in exclude_legs:
                continue

            arm = leg["arm"]

            mount = np.asarray(
                leg["mount"],
                dtype=float,
            )

            local_to_body = np.asarray(
                leg["local_to_body"],
                dtype=float,
            )

            old_coord = arm.get_coord()

            # Arm x, y, z -> internal x, z, y
            old_local = np.array([
                old_coord[0],
                old_coord[2],
                old_coord[1],
            ], dtype=float)

            foot_body_old = (
                mount
                + local_to_body @ old_local
            )

            current_foot_world = (
                old_position
                + old_R @ foot_body_old
            )

            new_target_body = (
                new_R.T
                @ (
                    current_foot_world
                    - new_position
                )
            )

            target_from_mount = (
                new_target_body
                - mount
            )

            local_target_leg = (
                local_to_body.T
                @ target_from_mount
            )

            target_coord = np.array([
                local_target_leg[0],
                local_target_leg[2],
                local_target_leg[1],
            ], dtype=float)

            arm.set_coord(
                float(target_coord[0]),
                float(target_coord[1]),
                float(target_coord[2]),
            )

        self.R = new_R
        self.position = new_position

    def set_rotation(
        self,
        roll=0,
        pitch=0,
        yaw=0,
        exclude_legs=None,
    ):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        new_R = (
            rot_z(yaw)
            @ rot_y(pitch)
            @ rot_x(roll)
        )

        self._apply_transform(
            new_R=new_R,
            exclude_legs=exclude_legs,
        )

    def set_rotation_local(
        self,
        roll=0,
        pitch=0,
        yaw=0,
        exclude_legs=None,
    ):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        delta_R = (
            rot_z(yaw)
            @ rot_y(pitch)
            @ rot_x(roll)
        )

        new_R = self.R @ delta_R

        self._apply_transform(
            new_R=new_R,
            exclude_legs=exclude_legs,
        )

    def set_position(
        self,
        x=0,
        y=0,
        z=0,
        exclude_legs=None,
    ):
        new_position = np.array([
            x,
            y,
            z,
        ], dtype=float)

        self._apply_transform(
            new_position=new_position,
            exclude_legs=exclude_legs,
        )

    def move(
        self,
        x=0,
        y=0,
        z=0,
        exclude_legs=None,
    ):
        movement_world = np.array([
            x,
            y,
            z,
        ], dtype=float)

        new_position = (
            self.position
            + movement_world
        )

        self._apply_transform(
            new_position=new_position,
            exclude_legs=exclude_legs,
        )

    def move_local(
        self,
        x=0,
        y=0,
        z=0,
        exclude_legs=None,
    ):
        movement_local = np.array([
            x,
            y,
            z,
        ], dtype=float)

        movement_world = (
            self.R
            @ movement_local
        )

        new_position = (
            self.position
            + movement_world
        )

        self._apply_transform(
            new_position=new_position,
            exclude_legs=exclude_legs,
        )

    def reset_rotation(
        self,
        exclude_legs=None,
    ):
        self._apply_transform(
            new_R=np.eye(3, dtype=float),
            exclude_legs=exclude_legs,
        )

    def reset_position(
        self,
        exclude_legs=None,
    ):
        self._apply_transform(
            new_position=np.zeros(
                3,
                dtype=float,
            ),
            exclude_legs=exclude_legs,
        )

    def reset_transform(
        self,
        exclude_legs=None,
    ):
        self._apply_transform(
            new_R=np.eye(3, dtype=float),
            new_position=np.zeros(
                3,
                dtype=float,
            ),
            exclude_legs=exclude_legs,
        )

    def get_position(self):
        return self.position.copy()

    def get_rotation_matrix(self):
        return self.R.copy()