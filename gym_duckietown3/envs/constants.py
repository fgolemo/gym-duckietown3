DEBUG = False
""" Use with care. Slows down simulation dramatically.
    And only works locally (i.e. you need a non-fake XServer)
"""

PATH_TO_URDF = "../assets/urdf/"

class CamParams(object):
    cameraUp = [0, 0, 1]
    camDistance = 4
    pixelWidth = 320 # for best gym-compatibility this should be set to 84 in practice
    pixelHeight = 200 # this too: set to 84 in production
    nearPlane = 0.01
    farPlane = 100
    fov = 60 # this should be tuned to be close to the actual robot's FOV (or randomized)

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

PHYSICS_STEPS_PER_STEP = 3
""" If this is bigger than 1, for each call to env.step() it will
    run multiple steps of the simulation.
"""

DISTANCE_TO_TARGET_EPSILON = 0.1
""" Small float value. If the distance between the car's base and 
    the target coordinates is smaller than this value, the task 
    is considered solved in the DistanceToTarget environments.
"""
