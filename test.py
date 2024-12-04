# test_env_registration.py

import gymnasium as gym
import gym_pollux  # Ensure this import is present to trigger registration

print("Available environments:")
for env_spec in gym.envs.registry.values():
    print(f"{env_spec.namespace}/{env_spec.name}, version={env_spec.version}")
