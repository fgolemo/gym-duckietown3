from gym.envs.registration import register

"""
simple 3 tiles straight road
reward: distance to target 
randomness: starting position (within a circle) and starting orientation (within a range)
"""
register(
    id='Duckiesim-StraightRoad-CPU-v0',
    entry_point='gym_duckietown3.envs:StraightEnv',
    max_episode_steps=1000,
    # reward_threshold=0.0,
    kwargs={'renderer': 'CPU'}
)

register(
    id='Duckiesim-StraightRoad-GPU-v0',
    entry_point='gym_duckietown3.envs:StraightEnv',
    max_episode_steps=1000,
    # reward_threshold=0.0,
    kwargs={'renderer': 'GPU'}
)

"""
simple 3 tiles straight road (same as above, but simpler)
reward: distance to target 
randomness: starting orientation (within a range of +/- 20 degr)
"""
register(
    id='Duckiesim-StraightRoad-Simple-CPU-v0',
    entry_point='gym_duckietown3.envs:StraightEnv',
    max_episode_steps=1000,
    # reward_threshold=0.0,
    kwargs={'renderer': 'CPU', 'simple': True}
)

register(
    id='Duckiesim-StraightRoad-Simple-GPU-v0',
    entry_point='gym_duckietown3.envs:StraightEnv',
    max_episode_steps=1000,
    # reward_threshold=0.0,
    kwargs={'renderer': 'GPU', 'simple': True}
)
