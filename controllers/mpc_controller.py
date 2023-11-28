# Imports


# Constants
dt = 0.001


# Dynamics
def dynamics(robot, X, u):
    theta = X[:6].reshape(-1, 1)
    dtheta = X[6:].reshape(-1, 1)
    M = robot.get_mass_matrix(theta)
    C = robot.get_coriolis(theta, dtheta)
    G = robot.get_gravity(theta)

    # Get joint acceleration
    ddtheta = np.linalg.inv(M) @ (u - C @ dtheta - G)

    # Discretized output
    dX = np.vstack((dtheta, ddtheta))

    X_next = X + dt * dX

    return X_next


# MPC controller
def controller(robot, planner, t, joint_positions, joint_velocities):
    X = np.vstack((joint_positions, joint_velocities))

    # Get entire trajectory from planner so that you can run MPC over the entire horizon. Or you can use a simple cubic trajectory 
    # which will add another planner script