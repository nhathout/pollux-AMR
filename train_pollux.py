import gymnasium as gym
import gym_pollux

from stable_baselines3 import PPO

def main():
    # Create the environment
    env = gym.make('PolluxSimpleEnv-v0')

    # Initialize the model
    model = PPO('MultiInputPolicy', env, verbose=1, device='cpu', ent_coef=0.1, learning_rate=1e-4)

    # Train the agent
    model.learn(total_timesteps=700000)

    # Save the trained model
    model.save("PPO_pollux_amr")

    print("Training completed and model saved.")

if __name__ == "__main__":
    main()
