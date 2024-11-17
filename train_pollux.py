import gymnasium as gym
import gym_pollux
from stable_baselines3 import PPO

def main():
    # Create the environment
    env = gym.make('PolluxSimpleEnv-v0')

    # Instantiate the agent
    model = PPO('MlpPolicy', env, verbose=1)

    # Train the agent
    model.learn(total_timesteps=50000)

    # Save the trained model
    model.save("ppo_pollux_amr")

    print("Training completed and model saved.")

if __name__ == "__main__":
    main()
