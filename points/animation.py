import math

#
# array of {x,y,z} points
#

oneStep = 30
y0 = -100
z0 = 65

stand_up_points = {
    "front_right": [
        {"x": 0, "y": -30, "z": z0},
        {"x": oneStep, "y": y0 + 20, "z": z0},
        {"x": oneStep, "y": y0, "z": z0+20},
    ],
    "front_left": [
        {"x": 0, "y": -30, "z": z0},
        {"x": oneStep, "y": y0 + 20, "z": z0},
        {"x": oneStep, "y": y0, "z": z0+20},
    ],
    "back_right": [
        {"x": 0, "y": -30, "z": z0},
        {"x": oneStep, "y": y0 + 20, "z": z0},
        {"x": oneStep, "y": y0, "z": z0+20},
    ],
    "back_left": [
        {"x": 0, "y": -30, "z": z0},
        {"x": oneStep, "y": y0 + 20, "z": z0},
        {"x": oneStep, "y": y0, "z": z0+20},
    ],
}

K = 5
count = 30 * K
countUp = 15 * K
kY = 40
xInit = 0
oneStep = 25
S = oneStep * 3
#y0 = -80
y0 = -70
z0 = 70


dX = 15
dY = -8
dZ = 10

walk_points = {
    "front_left": [
        {"x": 2 * oneStep, "y": y0, "z": z0 - dZ, "count": count},#1
        {"x": oneStep, "y": y0, "z": z0 + dZ, "count": count},#2
        {"x": xInit, "y": y0, "z": z0 + dZ, "count": count},#3
        # UP
        {"x": 3 * oneStep, "y": y0 + kY, "z": z0, "count": countUp},#4
        {"x": 3 * oneStep, "y": y0, "z": z0, "count": count - countUp},#4
    ],

    "front_right": [
        {"x": xInit, "y": y0, "z": z0 + dZ, "count": count},#1
        ## UP
        {"x": 3 * oneStep, "y": y0 + kY, "z": z0 - dZ, "count": countUp},#2
        {"x": 3 * oneStep, "y": y0, "z": z0 - dZ, "count": count - countUp},#2
        {"x": 2 * oneStep, "y": y0, "z": z0 - dZ, "count": count},#3
        {"x": oneStep, "y": y0, "z": z0, "count": count},#4
    ],

    "back_left": [
        {"x": 2 * oneStep + dX, "y": y0 + dY, "z": z0 - dZ, "count": count},#1
        {"x": 3 * oneStep + dX, "y": y0 + dY, "z": z0 + dZ, "count": count},#2
        ## UP
        {"x": xInit + dX, "y": y0 + kY + dY, "z": z0 + dZ, "count": countUp},#3
        {"x": xInit + dX, "y": y0 + dY, "z": z0 + dZ, "count": count - countUp},#3
        {"x": oneStep + dX, "y": y0 + dY, "z": z0, "count": count},#4
    ],

    "back_right": [
        {"x": xInit + dX, "y": y0 + kY + dY/1.5, "z": z0 + dZ, "count": countUp},#1
        {"x": xInit + dX, "y": y0 + dY/1.5, "z": z0 + dZ, "count": count - countUp},#1
        {"x": oneStep + dX, "y": y0 + dY/1.5, "z": z0 - dZ, "count": count},#2
        {"x": 2 * oneStep + dX, "y": y0 + dY/1.5, "z": z0 - dZ, "count": count},#3
        {"x": 3 * oneStep + dX, "y": y0 + dY/1.5, "z": z0, "count": count},#4
    ],
}

def getX(a):
    return 2 * oneStep * math.cos((a / 180) * math.pi) - z0 * math.sin((a / 180) * math.pi)

def getZ(a):
    return 2 * oneStep * math.sin((a / 180) * math.pi) + z0 * math.cos((a / 180) * math.pi)


dYaw = 45/3
dZr = 15
dXr = 15
dX = 0
countMove = countUp/2

rotate_right_points =  {
    "front_left": [
        {"x": getX(dYaw * 1), "y": y0, "z": getZ(dYaw * 1), "count": count},#1
        {"x": getX(dYaw * 1) + dXr, "y": y0, "z": getZ(dYaw * 1) - dZr, "count": countMove},#1
        {"x": getX(dYaw * 2), "y": y0, "z": getZ(dYaw * 2), "count": count},#2
        {"x": getX(dYaw * 2) - dXr, "y": y0, "z": getZ(dYaw * 2) - dZr, "count": countMove},#2
        {"x": getX(dYaw * 3), "y": y0, "z": getZ(dYaw * 3), "count": count},#3
        {"x": getX(dYaw * 3) + dXr, "y": y0, "z": getZ(dYaw * 3) + dZr, "count": countMove},#3
        {"x": getX(dYaw * 0) + dXr, "y": y0 + kY, "z": getZ(dYaw * 0), "count": countUp},#4
        {"x": getX(dYaw * 0) - dXr, "y": y0, "z": getZ(dYaw * 0), "count": count - countUp},#4
        {"x": getX(dYaw * 0) - dXr, "y": y0, "z": getZ(dYaw * 0) - dZr, "count": countMove},#4
    ],

    "front_right": [
        {"x": getX(-dYaw * 3), "y": y0, "z": getZ(-dYaw * 3), "count": count},#1
        {"x": getX(-dYaw * 3) + dXr, "y": y0, "z": getZ(-dYaw * 3) + dZr, "count": countMove},#1
        {"x": getX(-dYaw * 0) + dXr, "y": y0 + kY, "z": getZ(-dYaw * 0), "count": countUp},#2
        {"x": getX(-dYaw * 0) - dXr, "y": y0, "z": getZ(-dYaw * 0), "count": count - countUp},#2
        {"x": getX(-dYaw * 0) - dXr, "y": y0, "z": getZ(-dYaw * 0) - dZr, "count": countMove},#2
        {"x": getX(-dYaw * 1), "y": y0, "z": getZ(-dYaw * 1), "count": count},#3
        {"x": getX(-dYaw * 1) + dXr, "y": y0, "z": getZ(-dYaw * 1) - dZr, "count": countMove},#3
        {"x": getX(-dYaw * 2), "y": y0, "z": getZ(-dYaw * 2), "count": count},#4
        {"x": getX(-dYaw * 2) - dXr, "y": y0, "z": getZ(-dYaw * 2) + dZr, "count": countMove},#4
    ],

    "back_left": [
        {"x": getX(-dYaw * 2) + dX, "y": y0, "z": getZ(-dYaw * 2), "count": count},#1
        {"x": getX(-dYaw * 2) - dXr + dX, "y": y0, "z": getZ(-dYaw * 2) - dZr, "count": countMove},#1
        {"x": getX(-dYaw * 3) + dX, "y": y0, "z": getZ(-dYaw * 3), "count": count},#2
        {"x": getX(-dYaw * 3) + dXr + dX, "y": y0, "z": getZ(-dYaw * 3) + dZr, "count": countMove},#2
        {"x": getX(-dYaw * 0) + dXr + dX, "y": y0 + kY, "z": getZ(-dYaw * 0), "count": countUp},#3
        {"x": getX(-dYaw * 0) - dXr + dX, "y": y0, "z": getZ(-dYaw * 0), "count": count - countUp},#3
        {"x": getX(-dYaw * 0) - dXr + dX, "y": y0, "z": getZ(-dYaw * 0) + dZr, "count": countMove},#3
        {"x": getX(-dYaw * 1) + dX, "y": y0, "z": getZ(-dYaw * 1), "count": count},#4
        {"x": getX(-dYaw * 1) + dXr + dX, "y": y0, "z": getZ(-dYaw * 1) - dZr, "count": countMove},#4
    ],

    "back_right": [
        {"x": getX(dYaw * 0) + dXr + dX, "y": y0 + kY, "z": getZ(dYaw * 0), "count": countUp},#1
        {"x": getX(dYaw * 0) - dXr + dX, "y": y0, "z": getZ(dYaw * 0), "count": count - countUp},#1
        {"x": getX(-dYaw * 0) - dXr + dX, "y": y0, "z": getZ(-dYaw * 0) + dZr, "count": countMove},#1
        {"x": getX(dYaw * 1) + dX, "y": y0, "z": getZ(dYaw * 1), "count": count},#2
        {"x": getX(dYaw * 1) + dXr + dX, "y": y0, "z": getZ(dYaw * 1) - dZr, "count": countMove},#2
        {"x": getX(dYaw * 2) + dX, "y": y0, "z": getZ(dYaw * 2), "count": count},#3
        {"x": getX(dYaw * 2) - dXr + dX, "y": y0, "z": getZ(dYaw * 2) + dZr, "count": countMove},#3
        {"x": getX(dYaw * 3) + dX, "y": y0, "z": getZ(dYaw * 3), "count": count},#4
        {"x": getX(dYaw * 3) + dXr + dX, "y": y0, "z": getZ(dYaw * 3) + dZr, "count": countMove},#4
    ],
}