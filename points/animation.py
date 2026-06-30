#
# array of {x,y,z} points
#

oneStep = 30
y0 = -100
z0 = 60

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
K = 10
count = 30 * K
countUp = 10 * K
kY = 50
xInit = 0
oneStep = 35
S = oneStep * 3
#y0 = -80
y0 = -60
z0 = 60


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