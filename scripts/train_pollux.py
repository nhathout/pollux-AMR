import sys
print("Python executable in train_pollux.py:", sys.executable)

import gymnasium as gym
print("gym module:", gym)
print("gym.__file__:", gym.__file__)

import gym_pollux
print("gym_pollux module:", gym_pollux)
print("gym_pollux.__file__:", getattr(gym_pollux, '__file__', 'No __file__ attribute'))

from stable_baselines3 import PPO

def main():
    # Create the environment
    print("Creating environment in train_pollux.py")
    env = gym.make('PolluxSimpleEnv-v0')  # Ensure the '-v0' suffix is included
    print("Environment created in train_pollux.py:", env)

    # Initialize the model
    model = PPO('MultiInputPolicy', env, verbose=1, device='cpu', ent_coef=0.1, learning_rate=1e-4)

    # Train the agent
    model.learn(total_timesteps=500000)

    # Save the trained model
    model.save("PPO_pollux_amr")

    print("Training completed and model saved.")

if __name__ == "__main__":
    main()
