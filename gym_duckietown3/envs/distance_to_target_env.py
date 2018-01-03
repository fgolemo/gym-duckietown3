import pybullet
import numpy as np

from gym_duckietown3.envs.constants import DISTANCE_TO_TARGET_EPSILON
from gym_duckietown3.envs.generic_env import GenericEnv


class DistanceToTargetEnv(GenericEnv):
    def __init__(self, renderer="CPU"):
        if not hasattr(self, "target"):
            raise Exception("you should set 'self.target=(x,y)' in "
                            "parent before initializing this class."
                            "See for example how the init is done in straight_env.py")

        super().__init__(renderer)

    def get_reward(self):
        car_pos, _ = pybullet.getBasePositionAndOrientation(self.robotId)

        # calculate distance between car's X/Y coords and target X/Y coords
        dist = np.linalg.norm(np.array(car_pos[:2]) - np.array(self.target))

        done = False
        if dist < DISTANCE_TO_TARGET_EPSILON:
            done = True

        return dist, done  # that's reward, done-ness
