# test_pollux.py

import matplotlib
matplotlib.use('TkAgg')  # Or another suitable backend
import matplotlib.pyplot as plt

import gymnasium as gym
import gym_pollux  # Import this before gym.make()

from stable_baselines3 import PPO, DQN, A2C

def main():
    # Choose the algorithm (should match the one used during training)
    algo = 'PPO'  # Change to 'DQN' or 'A2C' as needed

    # Create the environment
    env = gym.make('PolluxSimpleEnv-v0', render_mode='human')  # Use the updated ID

    # Load the trained model
    if algo == 'PPO':
        model = PPO.load(f"{algo}_pollux_amr")
    elif algo == 'DQN':
        model = DQN.load(f"{algo}_pollux_amr")
    elif algo == 'A2C':
        model = A2C.load(f"{algo}_pollux_amr")
    else:
        raise ValueError(f"Unsupported algorithm: {algo}")

    # Run multiple episodes
    num_episodes = 20
    for episode in range(num_episodes):
        obs, info = env.reset()
        done = False
        total_reward = 0

        while not done:
            # Predict the action
            action, _states = model.predict(obs, deterministic=True)

            # Take the action in the environment
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            total_reward += reward

            # Render the environment
            env.render()

            if done:
                print(f"Episode {episode + 1} finished with total reward: {total_reward}")
                break

    # Keep the window open
    plt.show(block=True)

    env.close()

if __name__ == "__main__":
    main()
