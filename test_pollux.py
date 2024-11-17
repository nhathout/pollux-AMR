import gymnasium as gym
import gym_pollux
from stable_baselines3 import PPO

def main():
    # Create the environment
    env = gym.make('PolluxSimpleEnv-v0')

    # Load the trained model
    model = PPO.load("ppo_pollux_amr")

    # Reset the environment
    obs, info = env.reset()

    for _ in range(100):
        # Predict the action
        action, _states = model.predict(obs, deterministic=True)

        # Take the action in the environment
        obs, reward, terminated, truncated, info = env.step(action)

        # Render the environment
        env.render()

        if terminated or truncated:
            print("Episode finished.")
            break

    env.close()

if __name__ == "__main__":
    main()
