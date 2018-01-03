import gym
import numpy as np
import time

import pybullet
from gym import spaces

from gym_duckietown3.envs.constants import DEBUG, PATH_TO_URDF
from gym_duckietown3.road_layout import RoadLayout
from gym_duckietown3.utils import get_point_from_circle_distribution


class GenericEnv(gym.Env):
    def __init__(self, renderer="CPU", resolution=(84, 84), max_torque=10):
        self.renderer = renderer
        self.render_resolution = resolution

        if DEBUG:
            self.physicsClient = pybullet.connect(p.GUI)  # use this only in case of em
        else:
            self.physicsClient = pybullet.connect(p.DIRECT)

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

        if self.map is None:
            self.map = RoadLayout()

        self.generate_map(self.map)

    def _step(self, a):
        # TODO
        pass

    def _reset(self):
        # TODO
        pass

    def _seed(self, seed=None):
        # TODO should prolly force overwriting in child classes here (notimplemented exception)
        pass

    def _render(self, mode='human', close=False):
        # TODO
        pass

    def _close(self):
        # TODO
        pass

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
                _ = pybullet.loadURDF("../assets/urdf/road/road_{}.urdf".format(road_urdf), road_pos, road_orientation)

    def spawn_car(self, pos=(0, 0, 0), orn=(90, 0, 0)):
        """ Puts the robot into the world

        :param pos: x, y, radius of spawn circle
                    (if radius is 0, position is NOT random)
        :param orn: starting orientation, random range (negative and positive),
                    (if negative and positiverange  is 0, orientation is NOT random)
        :return: void
        """

        if pos[2] == 0:
            start_pos = [pos[0], pos[1], .01]
        else:
            rand_x, rand_y = get_point_from_circle_distribution(*pos)
            start_pos = [rand_x, rand_y, .01]

        if orn[1] == 0 and orn[2] == 0:
            start_orn = [0, 0, np.deg2rad(orn[0])]
        else:
            rand_angle = np.random.uniform(orn[0] + orn[1], orn[0] + orn[2])
            start_orn = [0, 0, np.deg2rad(rand_angle)]

        self.robotId = pybullet.loadURDF(PATH_TO_URDF + "robot/robot_clean.urdf", start_pos, start_orn)
