
from gym.envs.registration import register

"""
simple 3 tiles straight road
reward: distance to target 
randomness: starting position (within a circle) and starting orientation (within a range)
"""
register(
    id='Duckiesim-StraightRoad-CPU-v0',
    entry_point='gym_duckietown3.envs:StraightEnv',
    # max_episode_steps=100, # TODO add this back later when we have an estimate
    # reward_threshold=0.0,
    kwargs={'renderer': 'CPU'}
)

register(
    id='Duckiesim-StraightRoad-GPU-v0',
    entry_point='gym_duckietown3.envs:StraightEnv',
    # max_episode_steps=100, # TODO add this back later when we have an estimate
    # reward_threshold=0.0,
    kwargs={'renderer': 'GPU'}
)