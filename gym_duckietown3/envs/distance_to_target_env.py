import gym
import numpy as np
import time
from gym import spaces

from gym_duckietown3.envs.generic_env import GenericEnv


class DistanceToTargetEnv(GenericEnv):
    def __init__(self, renderer="CPU"):
        super().__init__(renderer)

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
