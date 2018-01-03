import gym
import numpy as np
import time
from gym import spaces

from gym_duckietown3.envs.generic_env import GenericEnv


class DistanceToTargetEnv(GenericEnv):
    def __init__(self, renderer="CPU"):
        super().__init__(renderer)

    def get_reward(self):
        # TODO implement this, based on distance from robot to self.target
        return 0, False  # that's reward, done-ness
