import gymnasium as gym
import gym_pollux
from stable_baselines3 import PPO
import torch

def main():
    # Create the environment
    env = gym.make('PolluxSimpleEnv-v0')

    # Initialize the model
    model = PPO('MultiInputPolicy', env, verbose=1, device='cpu', ent_coef=0.1, learning_rate=1e-4)

    # Train the agent
    model.learn(total_timesteps=700000)

    # Save in both formats:
    model.save("PPO_pollux_amr")  # SB3 format
    torch.save(model.policy.state_dict(), "PPO_pollux_amr.pth")  # PyTorch format

    print("Training completed. Saved:")
    print("- SB3 model: PPO_pollux_amr.zip")
    print("- PyTorch weights: PPO_pollux_amr.pth")

if __name__ == "__main__":
    main()
