import gym
import time

from gym_duckietown3.envs.distance_to_target_env import DistanceToTargetEnv
from gym_duckietown3.maps.straight_line_3x1 import MapStraightLine3x1


class StraightEnv(DistanceToTargetEnv):
    def __init__(self, renderer):
        self.map = MapStraightLine3x1()

        self.target = (0, 4)  # center of topmost tile, this is where the reward is
        start_orientation = (90, -45, 45)  # starting orientation with random range of +/- 45 deg
        start_position = (0, 0, .75)  # center x,y and radius of spawn circle

        super().__init__(renderer)

        self.spawn_car(start_position, start_orientation)


if __name__ == '__main__':
    import gym_duckietown3  # mandatory for next line

    env = gym.make("Duckiesim-StraightRoad-CPU-v0")
    env.reset()
    for i in range(100):
        env.render("human")
        action = env.action_space.sample()
        obs, reward, done, misc = env.step(action)
        print(action, obs.shape, reward, done)

        time.sleep(.1)
