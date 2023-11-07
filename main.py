from models.robot_env import RobotEnv
from models.robot_env import simulate_robot, simulate_robot_real_time
from planners.adaptive_planner import TrajectoryPlanner
from controllers.impedance_controller import controller
from utils import functions as f
from utils.dynamixel_utils import MotorController
import pinocchio as pin
import numpy as np


package_dirs = "/home/chris/Controllers/MCRL_urdf/"
urdf = package_dirs + "gazebo_touchscreen_robot_no_linear.urdf"
pin_robot = pin.RobotWrapper.BuildFromURDF(urdf, package_dirs)

robot = RobotEnv(pin_robot)
robot.start_visualizer()

motor_control = MotorController()

if __name__ == "__main__":
    T = 5.
    X_A = np.array([[0.0035],[-0.0021001],[0.46995]])
    X_B = np.array([[0.3], [0.3], [0.3]])
    end_effector_goal2 = np.array([[0.3], [0.5],[0.9]])

    robot.show_target(X_B.flatten())

    planner = TrajectoryPlanner(X_A, X_B, T)

    simulate_robot_real_time(robot, planner, controller, motor_control)
    #simulate_robot(robot, planner, controller)
