DEBUG = True
""" Use with care. Slows down simulation dramatically.
    And only works locally (i.e. you need a non-fake XServer)
"""

PATH_TO_URDF = "../assets/urdf/"

class CamParams(object):
    cameraUp = [0, 0, 1]
    camDistance = 4
    pixelWidth = 320
    pixelHeight = 200
    nearPlane = 0.01
    farPlane = 100
    fov = 60

CAM_PARAMS = CamParams()

LIGHT_DIRECTION = [1, 1, 1]
""" This is global directional light.
    I.e. every object has a light source this vector away from it's highest point
"""

SHADOW = 1
""" Shadow intensity (1 = full hard shadow)
    anything other than 1 or 0 will slow down rendering
"""

class RobotControl(object):
    wheel_max_speed_right = 15
    wheel_max_speed_left = 15
    max_force = 10

ROBOT_CONTROL = RobotControl()
