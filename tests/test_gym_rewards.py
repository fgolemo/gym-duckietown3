import gym

gyms = [
    'FrozenLake-v0',
    'Reacher-v1',
    'CartPole-v0',
    # 'LunarLander-v2',
    'Go9x9-v0',
    'SpaceInvaders-v0',
    'Copy-v0'
]

for env_name in gyms:
    env = gym.make(env_name)
    rewards = []
    env.reset()
    for _ in range(500):
        _, rew, done, _ = env.step(env.action_space.sample())
        if done:
            break
        rewards.append(rew)

    print("{}, Min: {}, Max: {}".format(env_name, min(rewards), max(rewards)))
