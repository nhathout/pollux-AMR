# gym_pollux/envs/__init__.py

print("Executing gym_pollux/envs/__init__.py")
from gymnasium.envs.registration import register

print("Registering PolluxSimpleEnv-v0")

register(
    id='PolluxSimpleEnv-v0',
    entry_point='gym_pollux.envs.pollux_simple_env:PolluxSimpleEnv',
    max_episode_steps=100,
)
