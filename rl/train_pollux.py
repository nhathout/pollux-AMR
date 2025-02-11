import sys
import os
import gymnasium as gym
import gym_pollux
from stable_baselines3 import PPO
import torch

# Debugging setup
print(f"Python {sys.version}")
print(f"Working directory: {os.getcwd()}")

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
        batch_size=32,  # Reduced for Pi's memory
        n_steps=512     # Smaller rollout buffer
    )

    # Training with progress saving
    try:
        model.learn(
            total_timesteps=500000,
            callback=SaveOnBestTrainingRewardCallback()  # Add this class
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

class SaveOnBestTrainingRewardCallback(BaseCallback):
    def __init__(self, check_freq=1000, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.best_mean_reward = -np.inf

    def _on_step(self):
        if self.n_calls % self.check_freq == 0:
            # Save temporary model
            self.model.save("ppo_pollux_temp")
        return True

if __name__ == "__main__":
    main()