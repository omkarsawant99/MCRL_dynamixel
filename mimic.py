from models.robot_env import RobotEnv
from models.robot_env import mimic_robot
from planners.adaptive_planner import TrajectoryPlanner
from controllers.impedance_controller import controller
from utils import functions as f
from utils.dynamixel_utils import MotorController
import pinocchio as pin
import numpy as np


# MCRL robot
mcrl_package_dirs = "./urdf/MCRL_urdf/"
mcrl_urdf = mcrl_package_dirs + "gazebo_touchscreen_robot_no_linear.urdf"
mcrl_end_effector_name = "link_tip"
mcrl_pin_robot = pin.RobotWrapper.BuildFromURDF(mcrl_urdf, mcrl_package_dirs)

# UR16e robot
# ur16_package_dirs = "./urdf/UR16e_urdf/"
# ur16_urdf = ur16_package_dirs + "/urdf/eSeries_UR16e_15012020_URDF_ONLY.urdf"
# ur16_end_effector_name = "Wrist3"
# ur16_pin_robot = pin.RobotWrapper.BuildFromURDF(ur16_urdf, ur16_package_dirs)
# q_neutral = np.array([0., -1.57, 0., -1.57, 0., 0.])

# Test manipulator
test_package_dirs = "./urdf/Test_urdf/"
test_urdf = test_package_dirs + "manipulator.urdf"
test_end_effector_name = "link_3_v4_1"
test_pin_robot = pin.RobotWrapper.BuildFromURDF(test_urdf, test_package_dirs)


robot = RobotEnv(test_pin_robot, test_end_effector_name)
robot.start_visualizer()

motor_control = MotorController()

if __name__ == "__main__":
    mimic_robot(robot, motor_control)
