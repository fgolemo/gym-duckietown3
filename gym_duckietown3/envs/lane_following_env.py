import gym
import numpy as np
import time
from gym import utils, error, spaces

class LaneFollowingEnv(gym.Env):
    def __init__(self):
        #TODO
        self.metadata = {
            'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second': 50
        }
        self.obs_dim = 23

        self.action_space = spaces.Box(
            -np.ones(7),
            np.ones(7)
        )

        high = np.inf * np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        self._seed()

    def _step(self, a):
        # TODO
        pass


    def _reset(self):
        #TODO
        pass

    def _seed(self, seed=None):
        #TODO
        pass

    def _render(self, mode='human', close=False):
        #TODO
        pass

    def _close(self):
        #TODO
        pass

