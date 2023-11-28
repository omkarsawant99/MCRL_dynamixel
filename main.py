from models.robot_env import RobotEnv
from models.robot_env import simulate_robot, simulate_robot_real_time
from planners.adaptive_planner import TrajectoryPlanner
from controllers.impedance_controller import controller
from utils import functions as f
#from utils.dynamixel_utils import MotorController
import pinocchio as pin
import numpy as np


# MCRL robot
mcrl_package_dirs = "./urdf/MCRL_urdf/"
mcrl_urdf = mcrl_package_dirs + "gazebo_touchscreen_robot_no_linear.urdf"
mcrl_end_effector_name = "link_tip"
mcrl_pin_robot = pin.RobotWrapper.BuildFromURDF(mcrl_urdf, mcrl_package_dirs)

# UR16e robot
ur16_package_dirs = "./urdf/UR16e_urdf/"
ur16_urdf = ur16_package_dirs + "/urdf/eSeries_UR16e_15012020_URDF_ONLY.urdf"
ur16_end_effector_name = "Wrist3"
ur16_pin_robot = pin.RobotWrapper.BuildFromURDF(ur16_urdf, ur16_package_dirs)
q_neutral = np.array([0., -1.57, 0., -1.57, 0., 0.])

# Test manipulator
test_package_dirs = "./urdf/Test_urdf/"
test_urdf = test_package_dirs + "manipulator.urdf"
test_end_effector_name = "link_3_v4_1"
test_pin_robot = pin.RobotWrapper.BuildFromURDF(test_urdf, test_package_dirs)


robot = RobotEnv(mcrl_pin_robot, mcrl_end_effector_name)
robot.start_visualizer()

#motor_control = MotorController()

if __name__ == "__main__":
    T = 10.
    
    X_A = np.array([[0.0035],[-0.0021001],[0.46995]])
    #X_A = np.array([[0],[0],[0]])

    X_B = np.array([[0.3], [0.3], [0.3]])
    #X_B = np.array([[-0.4], [0.4], [0.4]])
    
    end_effector_goal2 = np.array([[0.3], [0.5],[0.9]])

    robot.show_target(X_B.flatten())

    planner = TrajectoryPlanner(X_A, X_B, T)

    #simulate_robot_real_time(robot, planner, controller, motor_control)
    simulate_robot(robot, planner, controller, disturbance_end_effector=True)
