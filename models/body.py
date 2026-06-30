import math
import numpy as np

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])

def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ])

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


class Body:
    def __init__(self, legs):
        self.legs = legs
        self.R = np.eye(3)
    def set_rotation_matrix(self, roll=0, pitch=0, yaw=0):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        self.R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


    def transform_leg_point(self, leg_name, x, y, z):
        leg = self.legs[leg_name]
        mount = leg["mount"]
        local_to_body = leg["local_to_body"]

        local_leg = np.array([
            x,
            z,
            y,
        ])

        foot_world = mount + local_to_body @ local_leg
        target_body = self.R.T @ foot_world
        target_from_mount = target_body - mount
        target_leg = local_to_body.T @ target_from_mount

        return np.array([
            target_leg[0],
            target_leg[2],
            target_leg[1],
        ])


    def set_leg_coord(self, leg_name, x, y, z):
        tx, ty, tz = self.transform_leg_point(leg_name, x, y, z)
        self.legs[leg_name]["arm"].set_coord(tx, ty, tz)
    def set_rotation(self, roll=0, pitch=0, yaw=0):
        old_R = self.R.copy()

        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        new_R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        for name, leg in self.legs.items():
            arm = leg["arm"]
            mount = leg["mount"]
            local_to_body = leg["local_to_body"]

            old_local = arm.get_coord()

            # Your arm uses x,z,y order
            old_local = np.array([
                old_local[0],
                old_local[2],
                old_local[1],
            ])

            # foot position in world using OLD body rotation
            current_foot_world = old_R @ (mount + local_to_body @ old_local)

            # convert that same world foot position into NEW body coordinates
            new_target_body = new_R.T @ current_foot_world

            # target relative to leg mount
            local_target_body = new_target_body - mount

            # convert body coordinates back to leg-local coordinates
            local_target_leg = local_to_body.T @ local_target_body

            # convert x,z,y back to your arm's x,y,z
            local_target_leg = np.array([
                local_target_leg[0],
                local_target_leg[2],
                local_target_leg[1],
            ])

            arm.set_coord(*local_target_leg)

        self.R = new_R

        self.R = new_R