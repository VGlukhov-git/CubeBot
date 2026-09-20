import math


def check_angle(angle):
    return angle is not None and not math.isnan(angle)


class Arm:
    def __init__(self, props):
        self.chA = props["chA"]
        self.chB = props["chB"]
        self.chC = props["chC"]
        self.device = props["device"]

        self.invert = props.get("invert", False)
        self.position_callback = props.get("position_callback")

        # Calibration offsets
        self.delta_a = props.get("delta_a", 0)
        self.delta_b = props.get("delta_b", 0)
        self.delta_c = props.get("delta_c", 0)

        # Arm geometry
        # A - length from foot to knee
        # B - length from knee to hip
        # C - length from hip to center
        self.A = props["A"]
        self.B = props["B"]
        self.C = props["C"]

        # Current local coordinates
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Current joint angles actually sent to motors
        self.j1 = 0.0
        self.j2 = 0.0
        self.j3 = 0.0

        # Movement interpolation
        self.dX = 0.0
        self.dY = 0.0
        self.dZ = 0.0
        self.count = 0

    def get_coord(self):
        return self.x, self.y, self.z

    def get_angles(self):
        return self.j1, self.j2, self.j3

    def get_state(self):
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "J1": float(self.j1),
            "J2": float(self.j2),
            "J3": float(self.j3),
        }

    def calc_betta(self, D):
        cos_b = (
            D * D
            + self.B * self.B
            - self.A * self.A
        ) / (2 * D * self.B)

        cos_b = max(-1.0, min(1.0, cos_b))

        return math.degrees(
            math.acos(cos_b)
        )

    def calc_yamma(self, D):
        cos_b = (
            self.A * self.A
            + self.B * self.B
            - D * D
        ) / (2 * self.A * self.B)

        cos_b = max(-1.0, min(1.0, cos_b))

        return math.degrees(
            math.acos(cos_b)
        )

    def calc_delta(self, x, y):
        return math.degrees(
            math.atan2(y, x)
        )

    def calc_omega(self, x, z):
        return math.degrees(
            math.atan2(z, x)
        )

    def set_ch(self, angle, channel):
        self.device.send_message(
            channel,
            angle,
        )

    def set_a(self, angle):
        self.set_ch(angle, self.chA)

    def set_b(self, angle):
        self.set_ch(angle, self.chB)

    def set_c(self, angle):
        self.set_ch(angle, self.chC)

    def set_state(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def set_coord(self, x, y, z):
        self.set_state(x, y, z)

        if self.position_callback:
            self.position_callback({
                "x": x,
                "y": y,
                "z": z,
            })

        xn = math.sqrt(x * x + z * z) - self.C
        dist = math.sqrt(xn * xn + y * y)

        betta = self.calc_betta(dist)
        yamma = self.calc_yamma(dist)
        delta = self.calc_delta(xn, y)

        if self.invert:
            omega = self.calc_omega(x, z)
        else:
            omega = 180 - self.calc_omega(x, z)

        final_yamma = 180 - yamma
        final_betta = 90 + betta + delta
        final_omega = omega

        # Raw robot joint angles
        self.j1 = float(final_yamma)
        self.j2 = float(final_betta)
        self.j3 = float(final_omega)

        # Angles actually sent to servos
        self.servo_j1 = self.j1 + self.delta_a
        self.servo_j2 = self.j2 + self.delta_b
        self.servo_j3 = self.j3 + self.delta_c

        if check_angle(self.servo_j1):
            self.set_a(self.servo_j1)
        else:
            print("Invalid yamma")

        if check_angle(self.servo_j2):
            self.set_b(self.servo_j2)
        else:
            print("Invalid betta")

        if check_angle(self.servo_j3):
            self.set_c(self.servo_j3)
        else:
            print("Invalid omega")
    def set_angle(self, a, b, c):
        """
        Set angles directly, mainly for testing.
        """

        joint_1 = (
            a + self.delta_a
        )

        joint_2 = (
            b + self.delta_b
        )

        if self.invert:
            joint_3 = (
                180
                - c
                - self.delta_c
            )
        else:
            joint_3 = (
                c
                + self.delta_c
            )

        self.set_a(joint_1)
        self.set_b(joint_2)
        self.set_c(joint_3)

    # Test / interpolation methods

    def set_position(
        self,
        x,
        y,
        z,
        t,
    ):
        self.dX = (
            x - self.x
        ) / t

        self.dY = (
            y - self.y
        ) / t

        self.dZ = (
            z - self.z
        ) / t

        self.count = t

    def next_state(self):
        if self.count > 0:
            self.x += self.dX
            self.y += self.dY
            self.z += self.dZ

            self.set_state(
                self.x,
                self.y,
                self.z,
            )

        self.count -= 1

        return self.count < 0

    def next(self):
        if self.count > 0:
            self.x += self.dX
            self.y += self.dY
            self.z += self.dZ

            self.set_coord(
                self.x,
                self.y,
                self.z,
            )

        self.count -= 1

        return self.count < 0