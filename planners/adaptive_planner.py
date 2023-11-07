import numpy as np

def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def tanh_scaled(x, scale=10):
    return 0.5 * (np.tanh(scale * x) + 1)


class TrajectoryPlanner:
    def __init__(self, X_A, X_B, T):
        self.X_A = X_A
        self.X_B = X_B
        self.T = T
        self.total_distance = np.linalg.norm(X_B - X_A)
        self.V_max = 2 * self.total_distance / T
        self.direction = (X_B - X_A) / np.linalg.norm(X_B - X_A)

    def get_next_state(self, current_pos, current_vel, dt):
        remaining_distance = np.linalg.norm(self.X_B - current_pos)

        # If very close to the target, stop.
        if remaining_distance <= 0.01:
            return self.X_B, np.array([[0.0], [0.0], [0.0]])

        # Desired direction based on current position
        desired_dir = (self.X_B - current_pos) / remaining_distance

        # Calculate the progress percentage
        progress = 1 - remaining_distance / self.total_distance

        # If progress is negative i.e. disturbance is greater than total_distance
        if progress < 0:
            self.total_distance = remaining_distance
            self.V_max = 2 * self.total_distance / self.T

        # Speed adjustment based on the sigmoid and tanh functions
        speed_scale = tanh_scaled(progress) * tanh_scaled(1 - progress)
        desired_speed = self.V_max * speed_scale

        new_vel = desired_speed * desired_dir
        new_pos = current_pos + new_vel * dt
        return new_pos, new_vel
