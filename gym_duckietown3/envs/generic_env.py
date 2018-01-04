import gym
import numpy as np
import matplotlib.pyplot as plt

import pybullet
from gym import spaces

from gym_duckietown3.envs.constants import DEBUG, PATH_TO_URDF, CAM_PARAMS, LIGHT_DIRECTION, SHADOW, ROBOT_CONTROL, \
    PHYSICS_STEPS_PER_STEP
from gym_duckietown3.road_layout import RoadLayout
from gym_duckietown3.utils import get_point_from_circle_distribution

ERROR_SHOULD_SPAWN_FIRST = "Child class hasn't spawned car yet. " \
                           "Look at straight_env.py for an example of how and " \
                           "when to spawn the car."


class GenericEnv(gym.Env):
    def __init__(self, renderer="CPU"):
        self.renderer = renderer
        self.render_resolution = (CAM_PARAMS.pixelWidth, CAM_PARAMS.pixelHeight)

        if DEBUG:
            self.physicsClient = pybullet.connect(pybullet.GUI)  # use this only in case of em
        else:
            self.physicsClient = pybullet.connect(pybullet.DIRECT)

        if self.renderer == "GPU":
            self.render_engine = pybullet.ER_BULLET_HARDWARE_OPENGL  # GPU, _might_ work headlessly
        else:
            self.render_engine = pybullet.ER_TINY_RENDERER  # CPU

        pybullet.setGravity(0, 0, -10)  # good enough

        self.metadata = {
            'render.modes': ['human', 'rgb_array'],
            # 'video.frames_per_second': 50 # IDK what this was used for in Mujoco envs
        }
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(self.render_resolution[0], self.render_resolution[1], 3))

        self.action_space = spaces.Box(
            -np.ones(2),
            np.ones(2)
        )

        self._seed()

        if not hasattr(self, 'cam_params') or self.cam_params is None:
            self.cam_params = CAM_PARAMS

        if not hasattr(self, 'robot_control') or self.robot_control is None:
            self.robot_control = ROBOT_CONTROL

        if not hasattr(self, 'map') or self.map is None:
            # if map wasn't defined in super class before init, then get demo map
            self.map = RoadLayout()

        self.generate_map(self.map)

        self.wheels = {"left": 3, "right": 2}  # by looking at output of pybullet.getJointInfo()

        self.plt_ax = None  # placeholder for the window if render="human"

        self.current_state_has_been_rendered = False
        """ this will be set to true if the current state has already been rendered,
            so that when using the step and human render function, there is no double-
            rendering
        """

        self.observation_buffer = None
        """ This is the buffer that is holding the last good rendering
        """

        self.car_spawned = False
        self.spawn_pos = None
        self.spawn_orn = None

    def _step(self, action):
        if not self.car_spawned:
            raise Exception(ERROR_SHOULD_SPAWN_FIRST)

        self.run_action(action)

        obs = self.get_observation()
        rew, done = self.get_reward()
        misc = self.get_misc()

        if not done:
            done = self.is_out_of_bounds()

        return obs, rew, done, misc

    def _reset(self):
        self.reset_position()
        obs = self.get_observation()
        return obs

    def _seed(self, seed=None):
        np.random.seed(seed)

    def _render(self, mode='human', close=False):
        if not self.current_state_has_been_rendered:
            obs = self.get_observation()
        else:
            obs = self.observation_buffer

        if mode == "human":
            # check if there is an existing window to render to
            if self.plt_ax is None:
                self.make_plt_window()

            self.plt_img.set_data(obs)
            self.plt_ax.plot([0])
            plt.pause(0.001)  # I found this necessary - otherwise no visible img
        else:
            # if render mode is not human, then we just return the observation
            return obs

    def _close(self):
        # TODO IDK - do we need this? Should maybe close the PLT window or something
        pybullet.disconnect()

    def get_reward(self):
        """ Calculate the reward based on the current task

        :return: reward as float value,
                 done as boolean (only if necessary for task, otherwise always False)
        """

        # this should be implemented by child classes
        raise NotImplementedError

    def get_misc(self):
        """ Get any task-specific misc information... only if applicable

        :return: anything really
        """

        # (optional) can be overwritten by task

        return None

    def denormalize_action(self, action_value, scaling):
        """ Because actions are normalized to range [0,1] this has to be scaled up
            to the max velocity of the wheel in positive and negative range
            E.g. if the actual wheel velocity range is [-10,10]
            then 0 becomes -10, 0.5 becomes 0, and 1 becomes +10.


        :param action_value: the action value for a single wheel in range [0,1]
        :param scaling: the max speed for this wheel
        :return: float scaled action
        """
        return action_value * scaling * 2 - scaling

    def run_action(self, action):
        ## left wheel

        left_wheel_action = self.denormalize_action(
            action[0],
            self.robot_control.wheel_max_speed_left
        )

        pybullet.setJointMotorControl2(
            self.robotId,
            self.wheels["left"],
            pybullet.VELOCITY_CONTROL,
            targetVelocity=left_wheel_action,
            force=self.robot_control.max_force
        )

        ## right wheel

        right_wheel_action = self.denormalize_action(
            action[1],
            self.robot_control.wheel_max_speed_right
        )

        pybullet.setJointMotorControl2(
            self.robotId,
            self.wheels["right"],
            pybullet.VELOCITY_CONTROL,
            targetVelocity=right_wheel_action,
            force=self.robot_control.max_force
        )

        ## finally run the physics engine
        for _ in range(PHYSICS_STEPS_PER_STEP):
            pybullet.stepSimulation()

    def generate_map(self, map):
        rm_height = map.map_rez[0]
        rm_width = map.map_rez[1]
        for row in range(rm_height):
            for col in range(rm_width):
                tile_type, tile_rotation = map.map_conf[rm_height - row - 1][col]

                if tile_type == 0:
                    continue

                if tile_type == 1:
                    road_urdf = "plain"
                else:
                    road_urdf = "turn"

                actual_x = (map.map_start[0] - (rm_height - row - 1)) * 2
                actual_y = (col - map.map_start[1]) * 2
                road_rotation = np.deg2rad(tile_rotation * 90)

                print(row, col, actual_x, actual_y)

                road_pos = [actual_y, actual_x, 0]
                road_orientation = pybullet.getQuaternionFromEuler([0, 0, road_rotation])
                _ = pybullet.loadURDF(PATH_TO_URDF + "road/road_{}.urdf".format(road_urdf), road_pos, road_orientation)

    def spawn_car(self, pos=(0, 0, 0), orn=(90, 0, 0)):
        """ Puts the robot into the world

        :param pos: x, y, radius of spawn circle
                    (if radius is 0, position is NOT random)
        :param orn: starting orientation, random range (negative and positive),
                    (if negative and positiverange  is 0, orientation is NOT random)
        :return: void
        """

        self.spawn_pos = pos
        self.spawn_orn = orn

        start_pos, start_orn = self.get_spawn_pos_orn(pos, orn)

        self.robotId = pybullet.loadURDF(
            PATH_TO_URDF + "robot/robot_clean.urdf",
            start_pos,
            pybullet.getQuaternionFromEuler(start_orn)
        )
        self.car_spawned = True

    def reset_position(self):
        if not self.car_spawned:
            raise Exception(ERROR_SHOULD_SPAWN_FIRST)

        start_pos, start_orn = self.get_spawn_pos_orn(self.spawn_pos, self.spawn_orn)
        pybullet.resetBasePositionAndOrientation(
            self.robotId,
            start_pos,
            pybullet.getQuaternionFromEuler(start_orn)
        )

    def get_spawn_pos_orn(self, pos, orn):
        if pos[2] == 0:
            # if last position item is 0, no randomness
            start_pos = [pos[0], pos[1], .01]
        else:
            # otherwise get random pos within circle of radius pos[2]
            rand_x, rand_y = get_point_from_circle_distribution(*pos)
            start_pos = [rand_x, rand_y, .01]

        if orn[1] == 0 and orn[2] == 0:
            # no randomness
            start_orn = [0, 0, np.deg2rad(orn[0])]
        else:
            # pull starting orientation randomly from range
            rand_angle = np.random.uniform(orn[0] + orn[1], orn[0] + orn[2])
            start_orn = [0, 0, np.deg2rad(rand_angle)]

        return start_pos, start_orn

    def get_observation(self):
        """ get the image from the robot's cam

        :return: Image as array (width, height, 3)
        """

        # get current position of cam and of cam target (both robot parts)
        cam_state = pybullet.getLinkState(self.robotId, 4)
        cam_pos = cam_state[0]

        cam_target_state = pybullet.getLinkState(self.robotId, 5)
        cam_target_pos = cam_target_state[0]

        # get view matrix, which is the view position and angle of the cam
        view_matrix = pybullet.computeViewMatrix(
            cam_pos,
            cam_target_pos,
            self.cam_params.cameraUp
        )

        # the aspect probably doesn't change over the course of one gym run... but you know
        aspect = self.cam_params.pixelWidth / self.cam_params.pixelHeight

        # this calculates where the pixels are being put
        projectionMatrix = pybullet.computeProjectionMatrixFOV(
            self.cam_params.fov,
            aspect,
            self.cam_params.nearPlane,
            self.cam_params.farPlane
        )

        # now actually render the image
        img_arr = pybullet.getCameraImage(
            self.cam_params.pixelWidth,
            self.cam_params.pixelHeight,
            view_matrix,
            projectionMatrix,
            shadow=SHADOW,
            lightDirection=LIGHT_DIRECTION,
            renderer=self.render_engine
        )

        # w = img_arr[0]  # width of the image, in pixels, unused
        # h = img_arr[1]  # height of the image, in pixels, unused
        rgb = np.array(img_arr[2])  # color data RGB
        # depth = img_arr[3]  # depth data, unused (but comes for free)

        self.observation_buffer = rgb[:, :, :3]  # IDK what the fourth dimension does here
        self.current_state_has_been_rendered = True

        return self.observation_buffer

    def make_plt_window(self):
        plt.ion()
        img = np.random.uniform(0, 255, (self.cam_params.pixelHeight, self.cam_params.pixelWidth, 3))
        self.plt_img = plt.imshow(img, interpolation='none', animated=True, label="blah")
        self.plt_ax = plt.gca()

    def is_out_of_bounds(self):
        # self.get_current_bounds()
        car_pos, _ = pybullet.getBasePositionAndOrientation(self.robotId)
        # a more sophisticated way of checking woul be to use self.is_valid_tile()

        if car_pos[2] <= 0.004:
            return True  # robot fell
        else:
            return False  # robot prolly on map

    def is_valid_tile(self, car_pos):
        map_row, map_col = self.get_current_tile(car_pos)
        out_of_bounds_vert = map_row < 0 or map_row > (self.map.map_rez[0] - 1)
        out_of_bounds_horz = map_col < 0 or map_col > (self.map.map_rez[1] - 1)

        if out_of_bounds_vert or out_of_bounds_horz:
            # then the robot is out of bounds and there is guaranteed to be no tile here
            return False
        else:
            # get the tile config here
            tile_type, _ = self.map.map_conf[map_row][map_col]
            if tile_type == 0:
                return False
        return True

    def get_current_tile(self, car_pos):
        map_row = self.map.map_start[0] - round(car_pos[0] / 2)
        map_col = round(car_pos[1] / 2) + self.map.map_start[1]
        return map_row, map_col
