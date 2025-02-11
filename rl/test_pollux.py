import sys
import matplotlib
matplotlib.use('Agg')  # Headless backend for Raspberry Pi
import matplotlib.pyplot as plt
import gymnasium as gym
import gym_pollux
from stable_baselines3 import PPO

def main():
    # Environment setup
    try:
        env = gym.make('PolluxSimpleEnv-v0', render_mode='human')
    except Exception as e:
        print(f"Environment creation failed: {str(e)}")
        sys.exit(1)

    # Model loading with fallback
    try:
        model = PPO.load("PPO_pollux_amr", env=env)
    except FileNotFoundError:
        print("Model not found, using random policy")
        model = None

    # Evaluation parameters
    num_episodes = 20
    results = []

    for episode in range(num_episodes):
        obs = env.reset()
        done = False
        total_reward = 0
        episode_data = []

        while not done:
            if model:
                action, _ = model.predict(obs, deterministic=True)
            else:
                action = env.action_space.sample()  # Random policy

            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            total_reward += reward
            episode_data.append(reward)

            if done:
                print(f"Episode {episode+1} reward: {total_reward}")
                results.append(total_reward)
                plt.figure(figsize=(10, 5))
                plt.plot(episode_data)
                plt.savefig(f"episode_{episode+1}.png")
                plt.close()
                break

    # Save summary
    plt.figure(figsize=(10,5))
    plt.plot(results)
    plt.title("Episode Rewards")
    plt.savefig("test_summary.png")
    print("Saved test results to test_summary.png")

    env.close()

if __name__ == "__main__":
    main()