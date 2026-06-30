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
        # deltas are used to calibrate angles for each arm
        self.delta_a = props.get("delta_a", 0)
        self.delta_b = props.get("delta_b", 0)
        self.delta_c = props.get("delta_c", 0)
        
        ## parameters
        ## A - length from foot to knee
        ## B - length from knee to hip
        ## C - length from hip to center
        self.A = props["A"]
        self.B = props["B"]
        self.C = props["C"]

        self.x = 0
        self.y = 0
        self.z = 0

        ##
        self.dX = 0
        self.dY = 0
        self.dZ = 0
        self.count = 0

    def get_coord(self):
        return self.x, self.y, self.z
    def calc_betta(self, D):
        cos_b = (D * D + self.B * self.B - self.A * self.A) / (2 * D * self.B)
        cos_b = max(-1.0, min(1.0, cos_b))
        return math.degrees(math.acos(cos_b))

    def calc_yamma(self, D):
        cos_b = (self.A * self.A + self.B * self.B - D * D) / (2 * self.A * self.B)
        cos_b = max(-1.0, min(1.0, cos_b))
        return math.degrees(math.acos(cos_b))

    def calc_delta(self, x, y):
        return math.degrees(math.atan2(y, x))

    def calc_omega(self, x, z):
        return math.degrees(math.atan2(z, x))

    def set_ch(self, angle, channel):
        self.device.send_message(channel, angle)

    def set_a(self, angle):
        self.set_ch(angle, self.chA)

    def set_b(self, angle):
        self.set_ch(angle, self.chB)

    def set_c(self, angle):
        self.set_ch(angle, self.chC)
    def set_state(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def set_coord(self, x, y, z):
        self.set_state(x, y, z)

        if self.position_callback:
            self.position_callback({"x": x, "y": y, "z": z})

        xn = math.sqrt(x * x + z * z) - self.C
        dist = math.sqrt(xn * xn + y * y)

        betta = self.calc_betta(dist)
        yamma = self.calc_yamma(dist)
        delta = self.calc_delta(xn, y)

        if self.invert:
            omega = self.calc_omega(x, z)
        else:
            omega = 180 - self.calc_omega(x, z)

        final_yamma =  180 -yamma
        final_betta = 90 + (betta + delta)
        final_omega = omega

        if check_angle(final_yamma):
            self.set_a(final_yamma + self.delta_a)
        else:
            print("Invalid yamma")

        if check_angle(final_betta):
            self.set_b(final_betta + self.delta_b)
        else:            
            print("Invalid betta")

        if check_angle(final_omega):
            self.set_c(final_omega + self.delta_c)
        else:            
            print("Invalid omega")

    def set_angle(self, a, b, c):
        # row method to set angles directly, used for testing
        self.set_a(a + self.delta_a)
        self.set_b(b + self.delta_b)
        final_c = 180 - c - self.delta_c if self.invert else c + self.delta_c
        self.set_c(final_c)

    ## Test methods
    def set_position(self, x, y, z, t):
        self.dX = (x - self.x) / t
        self.dY = (y - self.y) / t
        self.dZ = (z - self.z) / t
        self.count = t

    def next_state(self):
        if self.count > 0:
            self.x += self.dX
            self.y += self.dY
            self.z += self.dZ
            self.set_state(self.x, self.y, self.z)

        self.count -= 1
        return self.count < 0
    def next(self):
        if self.count > 0:
            self.x += self.dX
            self.y += self.dY
            self.z += self.dZ
            self.set_coord(self.x, self.y, self.z)

        self.count -= 1
        return self.count < 0