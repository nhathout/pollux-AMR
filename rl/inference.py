import gymnasium as gym
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy

def load_model(env, path):
    # Create new policy with same architecture
    policy = ActorCriticPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        lr_schedule=lambda _: 1e-4  # Match training config
    )
    
    # Load state dict
    policy.load_state_dict(torch.load(path, map_location='cpu'))
    return policy

def main():
    env = gym.make('PolluxSimpleEnv-v0')
    
    # Load SB3 model (option 1)
    # model = PPO.load("PPO_pollux_amr", env=env)
    
    # Load .pth directly (option 2)
    policy = load_model(env, "PPO_pollux_amr.pth")
    model = PPO(policy, env, device='cpu')
    
    obs, _ = env.reset()
    for _ in range(1000):
        action, _ = model.predict(obs)
        obs, _, done, _, _ = env.step(action)
        if done:
            obs = env.reset()

if __name__ == "__main__":
    main()