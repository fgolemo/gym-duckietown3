import gym
import time

from gym_duckietown3.envs.distance_to_target_env import DistanceToTargetEnv
from gym_duckietown3.maps.straight_line_3x1 import MapStraightLine3x1


class StraightEnv(DistanceToTargetEnv):
    def __init__(self, renderer, simple=False):
        self.map = MapStraightLine3x1()

        self.target = (0, 4)  # center of topmost tile, this is where the reward is

        if not simple:
            start_orientation = (90, -45, 45)  # starting orientation with random range of +/- 45 deg
            start_position = (0, 0, .75)  # center x,y and radius of spawn circle
        else:  # simple
            start_orientation = (90, -20, 20)  # starting orientation with random range of +/- 20 deg
            start_position = (0, 0, 0)  # center x,y of spawn, no randomness

        super().__init__(renderer)

        self.spawn_car(start_position, start_orientation)

    def get_reward(self):
        rew, done = super(StraightEnv, self).get_reward()
        # 5.1 is highest possible
        return 5.1 + rew, done


if __name__ == '__main__':
    import gym_duckietown3  # mandatory for next line

    print("Here's the normal env:")
    env = gym.make("Duckiesim-StraightRoad-CPU-v0")
    env.reset()
    for i in range(20):
        env.render("human")
        action = env.action_space.sample()
        obs, reward, done, misc = env.step(action)
        print(action, obs.shape, reward, done, misc)

        time.sleep(.1)

    env.reset()
    for i in range(20):
        env.render("human")
        action = env.action_space.sample()
        obs, reward, done, misc = env.step(action)
        print(action, obs.shape, reward, done, misc)

        time.sleep(.1)

    print ("...and now the simple version")
    env.close()
    env = gym.make("Duckiesim-StraightRoad-Simple-CPU-v0")
    env.reset()
    for i in range(20):
        env.render("human")
        action = env.action_space.sample()
        obs, reward, done, misc = env.step(action)
        print(action, obs.shape, reward, done, misc)

        time.sleep(.1)

    env.reset()
    for i in range(20):
        env.render("human")
        action = env.action_space.sample()
        obs, reward, done, misc = env.step(action)
        print(action, obs.shape, reward, done, misc)

        time.sleep(.1)
