import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
import numpy as np
import sys

DEBUG = True

if DEBUG:
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
else:
    physicsClient = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
p.setGravity(0, 0, -10)

## Road maker stuff

# TODO: this could prolly be done better with a config file
import importlib.util

spec = importlib.util.spec_from_file_location("roaf_configs", "../assets/road_configs/road_conf_01.py")
foo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(foo)
roadmap = foo.RoadMap()

rm_height = roadmap.road_rez[0]
rm_width = roadmap.road_rez[1]

for row in range(rm_height):
    for col in range(rm_width):
        tile_type, tile_rotation = roadmap.road_conf[rm_height - row - 1][col]

        if tile_type == 0:
            continue

        road_urdf = ""
        if tile_type == 1:
            road_urdf = "plain"
        else:
            road_urdf = "turn"

        actual_x = (roadmap.road_start[0] - (rm_height - row - 1)) * 2
        actual_y = (col - roadmap.road_start[1]) * 2
        road_rotation = np.deg2rad(tile_rotation * 90)

        print(row, col, actual_x, actual_y)

        road_pos = [actual_y, actual_x, 0]
        road_orientation = p.getQuaternionFromEuler([0, 0, road_rotation])
        road_id = p.loadURDF("../assets/urdf/road/road_{}.urdf".format(road_urdf), road_pos, road_orientation)

cubeStartPos = [0.5, 0, .2]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)])
robotId = p.loadURDF("../assets/urdf/robot/robot_clean.urdf", cubeStartPos, cubeStartOrientation)
for i in range(p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i))

if DEBUG:
    p.addUserDebugText("cam", [0, 0, 0.1], textColorRGB=[1, 0, 0], textSize=1.5, parentObjectUniqueId=robotId,
                       parentLinkIndex=4)
    p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=robotId, parentLinkIndex=4)
    p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=robotId, parentLinkIndex=4)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=robotId, parentLinkIndex=4)

#### camera stuff

cameraUp = [0, 0, 1]
camera_pos_relative_to_base = np.array([.175, 0, .175])  # from URDF
camDistance = 4
pixelWidth = 320
pixelHeight = 200
nearPlane = 0.01
farPlane = 100
fov = 60

# plt.ion()
# img = [[1,2,3]*50]*100#np.random.rand(200, 320)
# image = plt.imshow(img,interpolation='none',animated=True,label="blah")
# ax = plt.gca()

#### joint control stuff

if DEBUG:
    targetVelocityLeftSlider = p.addUserDebugParameter("wheelVelocityLeft", -20, 20, 0)
    targetVelocityRightSlider = p.addUserDebugParameter("wheelVelocityRight", -20, 20, 0)
    maxForceSlider = p.addUserDebugParameter("maxForce", 0, 20, 20)

wheels = {"left": 3, "right": 2}  # by looking at output of p.getJointInfo() in loop
# also they are inverted in the model (or named badly)

i = 0
frame_time = 0

while True:
    start = time.time()
    p.stepSimulation()
    car_pos, car_orientation = p.getBasePositionAndOrientation(robotId)
    car_orientation_euler = p.getEulerFromQuaternion(car_orientation)

    cam_state = p.getLinkState(robotId, 4)
    cam_pos = cam_state[0]
    # cam_orientation = p.getEulerFromQuaternion(cam_state[1])

    cam_target_state = p.getLinkState(robotId, 5)
    cam_target_pos = cam_target_state[0]

    if DEBUG:
        maxForce = p.readUserDebugParameter(maxForceSlider)
        targetVelocityLeft = p.readUserDebugParameter(targetVelocityLeftSlider)
        targetVelocityRight = p.readUserDebugParameter(targetVelocityRightSlider)

        ## left wheel
        p.setJointMotorControl2(robotId, wheels["left"], p.VELOCITY_CONTROL, targetVelocity=targetVelocityLeft,
                                force=maxForce)
        ## right wheel
        p.setJointMotorControl2(robotId, wheels["right"], p.VELOCITY_CONTROL, targetVelocity=targetVelocityRight,
                                force=maxForce)

    # viewMatrix = p.computeViewMatrixFromYawPitchRoll(car_pos, camDistance, yaw, pitch, roll, upAxisIndex)
    # yaw = cam_orientation[2]
    # pitch= cam_orientation[0]
    # cam_target = [
    #     np.cos(yaw)*np.cos(pitch),
    #     np.sin(yaw)*np.cos(pitch),
    #     np.sin(pitch)
    # ]

    view_matrix = p.computeViewMatrix(cam_pos, cam_target_pos, cameraUp)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)
    img_arr = p.getCameraImage(pixelWidth, pixelHeight, view_matrix, projectionMatrix, shadow=1,
                               lightDirection=[1, 1, 1], renderer=p.ER_TINY_RENDERER)
    ## Renderer options:
    ## p.ER_TINY_RENDERER - CPU-based renderer
    ## p.ER_BULLET_HARDWARE_OPENGL - OpenGL-renderer

    w = img_arr[0]  # width of the image, in pixels
    h = img_arr[1]  # height of the image, in pixels
    rgb = img_arr[2]  # color data RGB
    dep = img_arr[3]  # depth data

    # image.set_data(rgb)
    # ax.plot([0])
    # plt.pause(0.01)

    # time.sleep(.01)

    frame_time += time.time() - start
    i += 1
    if i == 10:
        i = 0
        fps = 1 / (float(frame_time) / 10)
        # print("{} fps".format(np.around(fps,2)), end='\r', flush=True)
        print("{} fps".format(np.around(fps, 2)))
        frame_time = 0

p.disconnect()


# TODO: load multiple instances at different positions
