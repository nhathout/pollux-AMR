from gymnasium.envs.registration import register
from gym_pollux.envs.pollux_simple_env import PolluxSimpleEnv

print("Registering PolluxSimpleEnv-v0")

register(
    id='PolluxSimpleEnv-v0',
    entry_point='gym_pollux.envs.pollux_simple_env:PolluxSimpleEnv',
    max_episode_steps=100,
)

