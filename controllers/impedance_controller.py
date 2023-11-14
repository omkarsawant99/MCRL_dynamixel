import numpy as np
import scipy.linalg as sl
import utils.functions as f

def controller(robot, planner, t, joint_positions, joint_velocities):
    """A robot controller at every time t, this controller is called by the simulator.
        It receives as input the current joint positions and velocities and
        needs to return a [6,1] vector of desired torque commands
    """

    desired_joint_positions = np.zeros([6,1])
    desired_joint_velocities = np.zeros([6,1])

    # here we will only use the D controller to inject small joint damping
    D = 0.1*np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    D_3 = 25
    K = 800

    #D = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    #D_3 = 2500
    #K = 50000

    ##TODO - implement gravity compensation and impedance control
    G = robot.get_gravity(joint_positions)
    T_ee_positions = robot.get_forward_kinematics(joint_positions)
    ee_positions = T_ee_positions[:, [3]]
    ee_positions = np.delete(ee_positions, 3, axis=0)


    T_theta_n = robot.get_forward_kinematics(joint_positions)
    p_theta_n = T_theta_n[:, [3]]
    p_theta_n = np.delete(p_theta_n, 3, axis=0)
    R_identity = np.identity(3)
    T_parallel2space_originbody_wrt_space = np.append(R_identity, p_theta_n, axis=1)
    T_parallel2space_originbody_wrt_space = np.append(T_parallel2space_originbody_wrt_space, [[0,0,0,1]], axis=0)
    J_space = robot.get_spatial_jacobian(joint_positions)
    J_required = f.getAdjoint(sl.inv(T_parallel2space_originbody_wrt_space)) @ J_space
    J_only_translation = np.delete(J_required, [0,1,2], axis=0)
    J_t = np.transpose(J_only_translation)

    ee_velocities = J_only_translation @ joint_velocities
    ee_positions = p_theta_n

    #ee_positions = ee_positions.reshape(-1, )
    #ee_velocities = ee_velocities.reshape(-1, )
    # Get desired end effector positions and velocities
    desired_ee_positions, desired_ee_velocities = planner.get_next_state(ee_positions, ee_velocities, dt=0.001)

    desired_ee_positions = desired_ee_positions.reshape(-1, 1)
    desired_ee_velocities = desired_ee_velocities.reshape(-1, 1)


    desired_joint_torques = J_t @(K*(desired_ee_positions - ee_positions)+(D_3)*(desired_ee_velocities - ee_velocities)) + G - (np.diag(D) @(joint_velocities))

    return desired_joint_torques
