import sys
import os
import gymnasium as gym
import gym_pollux
import numpy as np  # Added for np.inf
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback  # Added missing import
import torch

# Debugging setup
print(f"Python {sys.version}")
print(f"Working directory: {os.getcwd()}")

class SaveOnBestTrainingRewardCallback(BaseCallback):
    def __init__(self, check_freq=1000, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.best_mean_reward = -np.inf
        self.save_path = "best_model"

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            # Calculate mean reward
            if len(self.model.ep_info_buffer) > 0:
                mean_reward = np.mean([ep['r'] for ep in self.model.ep_info_buffer])
                
                # Save best model
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    self.model.save(f"{self.save_path}_best")
                    print(f"New best model with mean reward: {mean_reward:.2f}")
                
                # Save temporary checkpoint
                self.model.save(f"{self.save_path}_temp")
                
        return True

def main():
    # Environment setup
    try:
        env = gym.make('PolluxSimpleEnv-v0')
        print("Environment created successfully")
    except gym.error.Error as e:
        print(f"Environment creation failed: {str(e)}")
        sys.exit(1)

    # Model configuration
    model = PPO(
        'MultiInputPolicy',
        env,
        verbose=1,
        device='cpu',
        ent_coef=0.1,
        learning_rate=1e-4,
        batch_size=32, # 24 might be better on Pi
        n_steps=512 # 256 might be better on Pi
    )

    # Training with progress saving
    try:
        model.learn(
            total_timesteps=500000,
            callback=SaveOnBestTrainingRewardCallback(check_freq=1000) # 500 might be better on Pi
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted - saving final model...")

    # Save models
    model.save("PPO_pollux_amr")
    torch.save({
        'policy_state_dict': model.policy.state_dict(),
        'config': model.get_parameters()
    }, "PPO_pollux_amr.pth")

    print("Training completed. Saved:")
    print("- SB3 model: PPO_pollux_amr.zip")
    print("- PyTorch weights: PPO_pollux_amr.pth")

if __name__ == "__main__":
    main()