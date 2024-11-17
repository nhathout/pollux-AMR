import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random

class PolluxSimpleEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(PolluxSimpleEnv, self).__init__()
        # Define action and observation space
        # Actions: 0 - Move forward, 1 - Turn left, 2 - Turn right
        self.action_space = spaces.Discrete(3)

        # Observation space: [front_sensor, left_sensor, right_sensor]
        # Sensor values: 0 (cliff detected) to 1 (no cliff)
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(3,), dtype=np.float32)

        # Define the state
        self.state = None

        # Define the grid world (10x10)
        self.grid_size = 10
        self.grid = np.zeros((self.grid_size, self.grid_size))

        # Define the agent's position and orientation
        self.agent_pos = [5, 5]
        self.agent_dir = 0  # 0: Up, 1: Right, 2: Down, 3: Left

        # Define cliffs (edges of the grid)
        self.cliffs = []

        # Define obstacles
        self.obstacles = []

        # For rendering
        self.viewer = None

        # Initialize current step
        self.current_step = 0
        self.max_steps = 100  # Optional: maximum steps per episode

    def step(self, action):
        assert self.action_space.contains(action), f"{action} is not a valid action"

        self.current_step += 1

        # Update the agent's orientation
        if action == 1:  # Turn left
            self.agent_dir = (self.agent_dir - 1) % 4
        elif action == 2:  # Turn right
            self.agent_dir = (self.agent_dir + 1) % 4
        # Move forward
        elif action == 0:
            self._move_forward()

        # Debug: Print the agent's position and action
        print(f"Step: {self.current_step}, Action: {action}, Position: {self.agent_pos}, Direction: {self.agent_dir}")

        # Get sensor readings
        observation = self._get_observation()

        # Calculate reward
        reward = self._calculate_reward(observation)

        # Check if episode is terminated or truncated
        terminated = self._is_terminated(observation)
        truncated = self._is_truncated()

        info = {}

        return observation, reward, terminated, truncated, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.current_step = 0

        # Reset agent's position and orientation
        self.agent_pos = [random.randint(1, self.grid_size - 2), random.randint(1, self.grid_size - 2)]
        self.agent_dir = random.randint(0, 3)

        # Optionally, reset obstacles
        self.obstacles = []
        for _ in range(5):
            obstacle_pos = [random.randint(1, self.grid_size - 2), random.randint(1, self.grid_size - 2)]
            if obstacle_pos != self.agent_pos:
                self.obstacles.append(obstacle_pos)

        observation = self._get_observation()
        info = {}
        return observation, info

    def render(self, mode='human'):
        # Simple text-based rendering
        grid_display = np.full((self.grid_size, self.grid_size), ' ')
        grid_display[self.agent_pos[1]][self.agent_pos[0]] = 'A'  # Agent

        # Display obstacles
        for obs in self.obstacles:
            grid_display[obs[1]][obs[0]] = 'O'

        # Display grid
        print('\n'.join([''.join(row) for row in grid_display]))
        print()

    def close(self):
        pass

    def _move_forward(self):
        # Determine the new position based on the current orientation
        if self.agent_dir == 0:  # Up
            new_pos = [self.agent_pos[0], self.agent_pos[1] - 1]
        elif self.agent_dir == 1:  # Right
            new_pos = [self.agent_pos[0] + 1, self.agent_pos[1]]
        elif self.agent_dir == 2:  # Down
            new_pos = [self.agent_pos[0], self.agent_pos[1] + 1]
        elif self.agent_dir == 3:  # Left
            new_pos = [self.agent_pos[0] - 1, self.agent_pos[1]]
        else:
            new_pos = self.agent_pos

        # Check for obstacles or cliffs
        if self._is_within_bounds(new_pos) and new_pos not in self.obstacles:
            self.agent_pos = new_pos  # Update position

    def _get_observation(self):
        # Simulate sensor readings
        front_sensor = self._sense_in_direction(self.agent_dir)
        left_sensor = self._sense_in_direction((self.agent_dir - 1) % 4)
        right_sensor = self._sense_in_direction((self.agent_dir + 1) % 4)
        return np.array([front_sensor, left_sensor, right_sensor], dtype=np.float32)

    def _sense_in_direction(self, direction):
        # Get the position one step ahead in the given direction
        if direction == 0:  # Up
            pos = [self.agent_pos[0], self.agent_pos[1] - 1]
        elif direction == 1:  # Right
            pos = [self.agent_pos[0] + 1, self.agent_pos[1]]
        elif direction == 2:  # Down
            pos = [self.agent_pos[0], self.agent_pos[1] + 1]
        elif direction == 3:  # Left
            pos = [self.agent_pos[0] - 1, self.agent_pos[1]]
        else:
            pos = self.agent_pos

        # Check for cliffs (edges of the grid)
        if not self._is_within_bounds(pos):
            return 0.0  # Cliff detected

        # Check for obstacles
        if pos in self.obstacles:
            return 0.0  # Obstacle detected

        return 1.0  # No cliff or obstacle

    def _calculate_reward(self, observation):
        # Reward function
        reward = 0.0
        if observation[0] == 0.0:
            reward -= 10.0  # Penalty for being close to a cliff or obstacle ahead
        else:
            reward += 1.0  # Reward for moving safely

        return reward

    def _is_terminated(self, observation):
        # Episode is terminated if the agent is about to fall off a cliff or after max steps
        if observation[0] == 0.0:
            return True
        else:
            return False

    def _is_truncated(self):
        # Truncate the episode if maximum steps reached
        return self.current_step >= self.max_steps

    def _is_within_bounds(self, pos):
        # Check if the position is within the grid bounds
        return 0 <= pos[0] < self.grid_size and 0 <= pos[1] < self.grid_size
