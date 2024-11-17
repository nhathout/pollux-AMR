from gymnasium.envs.registration import register

register(
    id='PolluxSimpleEnv-v0',
    entry_point='gym_pollux.envs:PolluxSimpleEnv',
    max_episode_steps=100,  # Optional: specify max steps per episode
)

