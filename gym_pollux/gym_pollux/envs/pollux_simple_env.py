import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class PolluxSimpleEnv(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps': 10}

    def __init__(self, render_mode=None):
        super(PolluxSimpleEnv, self).__init__()
        self.render_mode = render_mode

        # Define action and observation space
        # Actions:
        # 0 - Move forward
        # 1 - Move forward while turning left
        # 2 - Move forward while turning right
        # 3 - Turn left by 45 degrees (without moving)
        # 4 - Turn right by 45 degrees (without moving)
        self.action_space = spaces.Discrete(5)

        # Observation space
        self.memory_size = 10  # Number of recent positions to include
        self.observation_space = spaces.Dict({
            'sensors': spaces.Box(low=0.0, high=1.0, shape=(5,), dtype=np.float32),
            'recent_positions': spaces.Box(low=0, high=9, shape=(self.memory_size, 2), dtype=np.int32)
        })

        # Define the grid world (10x10)
        self.grid_size = 10

        # Define the agent's position and orientation
        self.agent_pos = [5, 5]
        self.agent_dir = 0  # 0 to 7, representing 8 orientations at 45-degree increments

        # Direction deltas for 8 orientations
        self.direction_deltas = {
            0: (0, -1),   # Up
            1: (1, -1),   # Up-Right
            2: (1, 0),    # Right
            3: (1, 1),    # Down-Right
            4: (0, 1),    # Down
            5: (-1, 1),   # Down-Left
            6: (-1, 0),   # Left
            7: (-1, -1)   # Up-Left
        }

        # Define obstacles
        self.obstacles = []

        # For rendering
        self.fig = None
        self.ax = None
        self.agent_marker = None

        # Initialize current step
        self.current_step = 0
        self.max_steps = 200  # Increased steps per episode

        # Set of visited positions
        self.visited_positions = set()

        # Last action taken
        self.last_action = None

        # Visit counts
        self.visit_counts = {}

        # Object adjacent cells tracking
        self.object_adjacent_cells = {}
        self.object_cells_cleaned = set()

        # Coverage milestones
        self.coverage_milestones = [0.25, 0.5, 0.75, 1.0]
        self.coverage_rewards_given = set()

        # Loop memory for detecting loops
        self.loop_memory = []
        self.loop_memory_size = 10  # Number of recent positions to remember

    def step(self, action):
        assert self.action_space.contains(action), f"{action} is not a valid action"

        self.current_step += 1
        self.last_action = action

        previous_position = self.agent_pos.copy()

        # Update the agent's orientation and position
        if action == 0:  # Move forward
            self._move_forward()
        elif action == 1:  # Move forward while turning left
            self.agent_dir = (self.agent_dir - 1) % 8
            self._move_forward()
        elif action == 2:  # Move forward while turning right
            self.agent_dir = (self.agent_dir + 1) % 8
            self._move_forward()
        elif action == 3:  # Turn left by 45 degrees (without moving)
            self.agent_dir = (self.agent_dir - 1) % 8
        elif action == 4:  # Turn right by 45 degrees (without moving)
            self.agent_dir = (self.agent_dir + 1) % 8

        # Update visit counts
        position_tuple = tuple(self.agent_pos)
        self.visit_counts[position_tuple] = self.visit_counts.get(position_tuple, 0) + 1

        # Add current position to visited positions
        self.visited_positions.add(position_tuple)

        # Update loop memory
        self.loop_memory.append(self.agent_pos.copy())

        # Get sensor readings
        observation = self._get_observation()

        # Check for collision
        collision = self._check_collision()

        # Update object adjacent cells
        self._update_object_adjacent_cells()

        # Calculate reward
        reward = self._calculate_reward(observation, collision, previous_position)

        # Check if episode is terminated or truncated
        terminated = self._is_terminated(collision)
        truncated = self._is_truncated()

        info = {}

        return observation, reward, terminated, truncated, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.current_step = 0
        self.visited_positions = set()
        self.visit_counts = {}
        self.object_adjacent_cells = {}
        self.object_cells_cleaned = set()
        self.coverage_rewards_given = set()
        self.loop_memory = []

        # Reset agent's position and orientation
        self.agent_pos = [random.randint(1, self.grid_size - 2), random.randint(1, self.grid_size - 2)]
        self.agent_dir = random.randint(0, 7)  # 0 to 7 for 8 orientations

        # Reset obstacles
        self.obstacles = []
        num_obstacles = random.randint(3, 5)
        for _ in range(num_obstacles):
            obstacle_pos = [random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1)]  # Allow obstacles on edge
            if obstacle_pos != self.agent_pos and obstacle_pos not in self.obstacles:
                self.obstacles.append(obstacle_pos)

        # Initialize object adjacent cells
        self._initialize_object_adjacent_cells()

        # Clear the figure for rendering
        if self.fig:
            plt.close(self.fig)
            self.fig = None

        observation = self._get_observation()
        info = {}
        return observation, info

    def render(self):
        if self.render_mode == 'human':
            if self.fig is None:
                self.fig, self.ax = plt.subplots()
                plt.ion()
                self.ax.set_xlim(-0.5, self.grid_size - 0.5)
                self.ax.set_ylim(-0.5, self.grid_size - 0.5)
                self.agent_marker = patches.Circle((0, 0), 0.3, color='blue')
                self.ax.add_patch(self.agent_marker)

                # Draw obstacles
                for obs in self.obstacles:
                    obstacle_marker = patches.Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='red')
                    self.ax.add_patch(obstacle_marker)

                # Set grid lines
                self.ax.set_xticks(range(self.grid_size))
                self.ax.set_yticks(range(self.grid_size))
                self.ax.grid(True)
                self.ax.invert_yaxis()

            # Update agent position
            self.agent_marker.center = (self.agent_pos[0], self.agent_pos[1])

            # Draw the path
            if len(self.visited_positions) > 1:
                xs, ys = zip(*self.visited_positions)
                self.ax.plot(xs, ys, color='green', marker='.', linestyle='None', markersize=5)

            # Update the figure
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.05)  # Adjust pause time as needed

    def close(self):
        pass  # Do not close the figure here

    def _move_forward(self):
        # Determine the new position based on the current orientation
        delta = self.direction_deltas[self.agent_dir]
        new_pos = [self.agent_pos[0] + delta[0], self.agent_pos[1] + delta[1]]

        # Update position (allow moving into obstacles or out of bounds)
        self.agent_pos = new_pos

    def _get_observation(self):
        # Simulate sensor readings
        # Directions: left, front-left, front, front-right, right
        left_dir = (self.agent_dir - 2) % 8
        front_left_dir = (self.agent_dir - 1) % 8
        front_dir = self.agent_dir
        front_right_dir = (self.agent_dir + 1) % 8
        right_dir = (self.agent_dir + 2) % 8

        left_sensor = self._sense_in_direction(left_dir)
        front_left_sensor = self._sense_in_direction(front_left_dir)
        front_sensor = self._sense_in_direction(front_dir)
        front_right_sensor = self._sense_in_direction(front_right_dir)
        right_sensor = self._sense_in_direction(right_dir)

        sensors = np.array(
            [left_sensor, front_left_sensor, front_sensor, front_right_sensor, right_sensor],
            dtype=np.float32
        )

        # Recent positions
        loop_mem = self.loop_memory[-self.memory_size:]
        num_positions = len(loop_mem)

        # Initialize recent_positions with zeros
        recent_positions = np.zeros((self.memory_size, 2), dtype=np.int32)

        if num_positions > 0:
            # Convert loop_mem to a NumPy array
            temp_recent_positions = np.array(loop_mem, dtype=np.int32)
            # Ensure the array has shape (N, 2)
            if temp_recent_positions.ndim == 1:
                temp_recent_positions = temp_recent_positions.reshape(1, 2)
            # Insert recent positions into the recent_positions array
            recent_positions[-num_positions:] = temp_recent_positions

        return {'sensors': sensors, 'recent_positions': recent_positions}

    def _sense_in_direction(self, direction):
        # Get the position one step ahead in the given direction
        delta = self.direction_deltas[direction]
        pos = [self.agent_pos[0] + delta[0], self.agent_pos[1] + delta[1]]

        # Check for cliffs (edges of the grid)
        if not self._is_within_bounds(pos):
            return 0.0  # Cliff detected

        # Check for obstacles
        if pos in self.obstacles:
            return 0.0  # Obstacle detected

        return 1.0  # No cliff or obstacle

    def _calculate_reward(self, observation, collision, previous_position):
        # Reward function
        reward = 0.0

        if collision:
            reward -= 5.0  # Reduced penalty for collision
        else:
            position_tuple = tuple(self.agent_pos)
            visit_count = self.visit_counts[position_tuple]

            # Check if the agent has moved
            if self.agent_pos == previous_position:
                reward -= 1.0  # Penalty for not moving
            elif self.agent_pos in self.loop_memory[-self.memory_size:-1]:
                reward -= 2.0  # Penalty for loops
            else:
                # Reward for visiting new cells
                if visit_count == 1:
                    reward += 1.0  # Reward for visiting a new cell
                else:
                    # Encourage exploration of less-visited cells
                    exploration_bonus = 1.0 / (visit_count)
                    reward += exploration_bonus

                # Small reward for moving forward
                if self.last_action in [0, 1, 2]:
                    reward += 0.1

            # Encourage the agent to get close to obstacles
            if observation['sensors'][2] == 0.0:  # Front sensor detects obstacle
                reward += 0.5  # Reward for being close to an obstacle

            # Bonus for fully cleaning around objects
            bonus = self._check_cleaned_objects()
            reward += bonus * 0.1  # Scaled down bonus

            # Provide coverage milestone bonuses
            coverage_bonus = self._check_coverage_bonus()
            reward += coverage_bonus * 0.1  # Scaled down bonus

            # Slight penalty for turning actions to discourage unnecessary turns
            if self.last_action in [3, 4]:
                reward -= 0.05

        # Clip the reward to prevent extreme values
        reward = np.clip(reward, -5.0, 5.0)

        return reward

    def _check_coverage_bonus(self):
        # Calculate coverage percentage
        total_accessible_cells = (self.grid_size ** 2) - len(self.obstacles)
        coverage = len(self.visited_positions) / total_accessible_cells

        bonus = 0.0
        for milestone in self.coverage_milestones:
            if coverage >= milestone and milestone not in self.coverage_rewards_given:
                bonus += 1.0 * milestone  # Scaled down bonus
                self.coverage_rewards_given.add(milestone)

        return bonus

    def _is_terminated(self, collision):
        # Episode is terminated if the agent collides with an obstacle or goes out of bounds
        return collision

    def _is_truncated(self):
        # Truncate the episode if maximum steps reached
        return self.current_step >= self.max_steps

    def _is_within_bounds(self, pos):
        # Check if the position is within the grid bounds
        return 0 <= pos[0] < self.grid_size and 0 <= pos[1] < self.grid_size

    def _check_collision(self):
        # Check if the agent is out of bounds or has hit an obstacle
        if not self._is_within_bounds(self.agent_pos):
            return True  # Collision with boundary
        if self.agent_pos in self.obstacles:
            return True  # Collision with obstacle
        return False

    def _initialize_object_adjacent_cells(self):
        # Initialize adjacent cells for each obstacle
        for obs in self.obstacles:
            adj_cells = self._get_adjacent_cells(obs)
            self.object_adjacent_cells[tuple(obs)] = set(adj_cells)

    def _get_adjacent_cells(self, pos):
        # Get all valid adjacent cells around a position
        x, y = pos
        adjacent_positions = [
            (x - 1, y), (x + 1, y),
            (x, y - 1), (x, y + 1),
            (x - 1, y - 1), (x + 1, y - 1),
            (x - 1, y + 1), (x + 1, y + 1)
        ]
        valid_adjacent = [
            p for p in adjacent_positions if self._is_within_bounds(p) and p not in self.obstacles
        ]
        return valid_adjacent

    def _update_object_adjacent_cells(self):
        # Update the cleaning status of object adjacent cells
        for obs_pos, adj_cells in self.object_adjacent_cells.items():
            if obs_pos in self.object_cells_cleaned:
                continue  # Already cleaned around this object

            # Check if all adjacent cells have been visited
            if adj_cells.issubset(self.visited_positions):
                self.object_cells_cleaned.add(obs_pos)

    def _check_cleaned_objects(self):
        # Calculate bonus for cleaning around objects
        bonus = 0.0
        new_cleaned_objects = []
        for obs_pos in self.object_cells_cleaned:
            if obs_pos not in self.coverage_rewards_given:
                new_cleaned_objects.append(obs_pos)
                self.coverage_rewards_given.add(obs_pos)

        if new_cleaned_objects:
            bonus += 5.0 * len(new_cleaned_objects)  # Adjusted bonus per object

        return bonus
