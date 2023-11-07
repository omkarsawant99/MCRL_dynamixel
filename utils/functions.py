import numpy as np
import scipy.linalg as sl
import time


def vec_to_skew(w):
    v1 = w[0, 0]
    v2 = w[1, 0]
    v3 = w[2, 0]
    skew_sym_form = np.array([[0, -v3, v2], [v3, 0, -v1], [-v2, v1, 0]]);
    return skew_sym_form


def twist_to_skew(V):
    [w, v] = np.split(V, 2)
    w_ss = vec_to_skew(w)
    V_ss = np.append(w_ss, v, axis=1)
    V_ss = np.append(V_ss, [[0, 0, 0, 0]], axis=0)
    return V_ss


def exp_twist_bracket(V, t=1):
    V_ss = twist_to_skew(V)
    exp = sl.expm(V_ss * t)
    return exp


def inverseT(T):
    T = np.delete(T, 3, 0)
    R = T[:, [0, 1, 2]]
    p = T[:, [3]]

    invR = np.transpose(R)
    invp = -invR @ p
    invT = np.append(invR, invp, axis=1)
    invT = np.append(invT, [[0, 0, 0, 1]], axis=0)
    return invT


def getAdjoint(T):
    T = np.delete(T, 3, 0)
    R = T[:, [0, 1, 2]]
    p = T[:, [3]]

    # Creating zero matrix
    zeroes = np.zeros((3, 3))

    # Skew sym form of p and term [p]*R
    p_ss = vec_to_skew(p)
    p_mult_R = np.matmul(p_ss, R)

    # Combining matrices to form adjoint
    T1 = np.concatenate((R, zeroes), axis=1)
    T2 = np.concatenate((p_mult_R, R), axis=1)
    T_adj = np.concatenate((T1, T2), axis=0)
    return T_adj


def matrix_to_twist(V_ss):
    w_1 = V_ss[2][1]
    w_2 = -V_ss[2][0]
    w_3 = V_ss[1][0]
    v_1 = V_ss[0][3]
    v_2 = V_ss[1][3]
    v_3 = V_ss[2][3]
    V = np.array([[w_1], [w_2], [w_3], [v_1], [v_2], [v_3]])
    return V


# Forward kinematics

l_s1 = 0.1575
l_12 = 0.2025
l_23 = 0.2045
l_34 = 0.2155
l_45 = 0.1845
l_56 = 0.2155
l_67 = 0.0810
l_7b = 0.04
l_total = l_s1 + l_12 + l_23 + l_34 + l_45 + l_56 + l_67 + l_7b

M = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, l_total],
              [0, 0, 0, 1]])
S1 = np.array([[0], [0], [1], [0], [0], [0]])
S2 = np.array([[0], [1], [0], [-(l_s1 + l_12)], [0], [0]])
S3 = np.array([[0], [0], [1], [0], [0], [0]])
S4 = np.array([[0], [-1], [0], [(l_s1 + l_12 + l_23 + l_34)], [0], [0]])
S5 = np.array([[0], [0], [1], [0], [0], [0]])
S6 = np.array([[0], [1], [0], [-(l_s1 + l_12 + l_23 + l_34 + l_45 + l_56)], [0], [0]])
S7 = np.array([[0], [0], [1], [0], [0], [0]])

# Body screws
B7 = np.array([0, 0, 1, 0, 0, 0]).reshape(6, 1)
B6 = np.array([0, 1, 0, l_7b + l_67, 0, 0]).reshape(6, 1)
B5 = np.array([0, 0, 1, 0, 0, 0]).reshape(6, 1)
B4 = np.array([0, -1, 0, -(l_45 + l_56 + l_67 + l_7b), 0, 0]).reshape(6, 1)
B3 = np.array([0, 0, 1, 0, 0, 0]).reshape(6, 1)
B2 = np.array([0, 1, 0, (l_23 + l_34 + l_45 + l_56 + l_67 + l_7b), 0, 0]).reshape(6, 1)
B1 = np.array([0, 0, 1, 0, 0, 0]).reshape(6, 1)

def forward_kinematics(theta):
    t1 = theta[0]
    t2 = theta[1]
    t3 = theta[2]
    t4 = theta[3]
    t5 = theta[4]
    t6 = theta[5]
    t7 = theta[6]
    T_sh = sl.expm(twist_to_skew(S1) * t1) @ sl.expm(twist_to_skew(S2) * t2) @ sl.expm(
        twist_to_skew(S3) * t3) @ sl.expm(twist_to_skew(S4) * t4) @ sl.expm(twist_to_skew(S5) * t5) @ sl.expm(
        twist_to_skew(S6) * t6) @ sl.expm(twist_to_skew(S7) * t7) @ M
    return T_sh


# Space Jacobian
def get_space_jacobian(theta):
    t1 = theta[0]
    t2 = theta[1]
    t3 = theta[2]
    t4 = theta[3]
    t5 = theta[4]
    t6 = theta[5]
    t7 = theta[6]

    Js1 = S1
    Js2 = getAdjoint(sl.expm(twist_to_skew(S1) * t1)) @ S2
    Js3 = getAdjoint(sl.expm(twist_to_skew(S1) * t1) @ sl.expm(twist_to_skew(S2) * t2)) @ S3
    Js4 = getAdjoint(
        sl.expm(twist_to_skew(S1) * t1) @ sl.expm(twist_to_skew(S2) * t2) @ sl.expm(twist_to_skew(S3) * t3)) @ S4
    Js5 = getAdjoint(
        sl.expm(twist_to_skew(S1) * t1) @ sl.expm(twist_to_skew(S2) * t2) @ sl.expm(twist_to_skew(S3) * t3) @ sl.expm(
            twist_to_skew(S4) * t4)) @ S5
    Js6 = getAdjoint(
        sl.expm(twist_to_skew(S1) * t1) @ sl.expm(twist_to_skew(S2) * t2) @ sl.expm(twist_to_skew(S3) * t3) @ sl.expm(
            twist_to_skew(S4) * t4) @ sl.expm(twist_to_skew(S5) * t5)) @ S6
    Js7 = getAdjoint(
        sl.expm(twist_to_skew(S1) * t1) @ sl.expm(twist_to_skew(S2) * t2) @ sl.expm(twist_to_skew(S3) * t3) @ sl.expm(
            twist_to_skew(S4) * t4) @ sl.expm(twist_to_skew(S5) * t5) @ sl.expm(twist_to_skew(S6) * t6)) @ S7

    Js = np.append(Js1, Js2, axis=1)
    Js = np.append(Js, Js3, axis=1)
    Js = np.append(Js, Js4, axis=1)
    Js = np.append(Js, Js5, axis=1)
    Js = np.append(Js, Js6, axis=1)
    Js = np.append(Js, Js7, axis=1)

    return Js

def get_body_jacobian(theta):
    t1 = theta[0]
    t2 = theta[1]
    t3 = theta[2]
    t4 = theta[3]
    t5 = theta[4]
    t6 = theta[5]
    t7 = theta[6]

    Jb1 = getAdjoint(
        sl.expm(-1 * twist_to_skew(B7) * t7) @ sl.expm(-1 * twist_to_skew(B6) * t6) @ sl.expm(
            -1 * twist_to_skew(B5) * t5) @ sl.expm(
            -1 * twist_to_skew(B4) * t4) @ sl.expm(-1 * twist_to_skew(B3) * t3) @ sl.expm(
            -1 * twist_to_skew(B2) * t2)) @ B1
    Jb2 = getAdjoint(
        sl.expm(-1 * twist_to_skew(B7) * t7) @ sl.expm(-1 * twist_to_skew(B6) * t6) @ sl.expm(
            -1 * twist_to_skew(B5) * t5) @ sl.expm(
            -1 * twist_to_skew(B4) * t4) @ sl.expm(-1 * twist_to_skew(B3) * t3)) @ B2
    Jb3 = getAdjoint(
        sl.expm(-1 * twist_to_skew(B7) * t7) @ sl.expm(-1 * twist_to_skew(B6) * t6) @ sl.expm(
            -1 * twist_to_skew(B5) * t5) @ sl.expm(-1 * twist_to_skew(B4) * t4)) @ B3
    Jb4 = getAdjoint(
        sl.expm(-1 * twist_to_skew(B7) * t7) @ sl.expm(-1 * twist_to_skew(B6) * t6) @ sl.expm(
            -1 * twist_to_skew(B5) * t5)) @ B4
    Jb5 = getAdjoint(sl.expm(-1 * twist_to_skew(B7) * t7) @ sl.expm(-1 * twist_to_skew(B6) * t6)) @ B5
    Jb6 = getAdjoint(sl.expm(-1 *twist_to_skew(B7) * t7)) @ B6
    Jb7 = B7

    Jb = np.append(Jb1, Jb2, axis=1)
    Jb = np.append(Jb, Jb3, axis=1)
    Jb = np.append(Jb, Jb4, axis=1)
    Jb = np.append(Jb, Jb5, axis=1)
    Jb = np.append(Jb, Jb6, axis=1)
    Jb = np.append(Jb, Jb7, axis=1)

    return Jb


# J_plus with Tikhonov reg
def pinv(J):
    alpha = 10 ** (-3)
    I = np.identity(np.size(J, 0))
    J_T = np.transpose(J)
    J_plus = J_T @ (J @ J_T + alpha * I)
    return J_plus


def compute_IK_position(p_des, suppress_output = False):
    theta_n = np.array([[0], [0], [0], [0], [0], [0], [0]])
    k = 0
    e = 1
    while np.any(np.absolute(e) >= 10 ** (-4)) and k < 500:
        T_theta_n = forward_kinematics(theta_n)
        p_theta_n = T_theta_n[:, [3]]
        p_theta_n = np.delete(p_theta_n, 3, axis=0)
        R_identity = np.identity(3)
        T_parallel2space_originbody_wrt_space = np.append(R_identity, p_theta_n, axis=1)
        T_parallel2space_originbody_wrt_space = np.append(T_parallel2space_originbody_wrt_space, [[0, 0, 0, 1]],
                                                          axis=0)  # Why this frame?

        J_space = get_space_jacobian(theta_n)
        J_required = getAdjoint(sl.inv(T_parallel2space_originbody_wrt_space)) @ J_space
        J_only_translation = np.delete(J_required, [0, 1, 2], axis=0)

        J_plus = sl.pinv(J_only_translation)

        e = J_plus @ (p_des - p_theta_n)
        theta_next = theta_n + e
        theta_n = theta_next
        k = k + 1

    if not suppress_output:
        print('The error is', e)
        print('\n')
        print('Iterations = ', k)
        print('\n')
        print('The difference between IK position and desired position is \n', p_des - p_theta_n)
    return theta_n


def compute_IK_position_nullspace(p_des, theta0):
    theta_n = np.array([[0], [0], [0], [0], [0], [0], [0]])
    k = 0
    e = 100
    # while(round(e[0][0], 4) != 0 and  round(e[1][0], 4) != 0 and round(e[2][0], 4) != 0):
    # for j in range(500):
    while np.any(np.absolute(e) >= 10 ** (-4)) and k < 500:
        T_theta_n = forward_kinematics(theta_n)
        p_theta_n = T_theta_n[:, [3]]
        p_theta_n = np.delete(p_theta_n, 3, axis=0)
        R_identity = np.identity(3)
        T_parallel2space_originbody_wrt_space = np.append(R_identity, p_theta_n, axis=1)
        T_parallel2space_originbody_wrt_space = np.append(T_parallel2space_originbody_wrt_space, [[0, 0, 0, 1]],
                                                          axis=0)  # Why this frame?

        J_space = get_space_jacobian(theta_n)
        J_required = getAdjoint(sl.inv(T_parallel2space_originbody_wrt_space)) @ J_space
        J_only_translation = np.delete(J_required, [0, 1, 2], axis=0)

        J_plus = sl.pinv(J_only_translation)

        e = (J_plus @ (p_des - p_theta_n)) + ((I - (J_plus @ J_only_translation)) @ (theta0 - theta_n))
        theta_next = theta_n + e
        theta_n = theta_next
        k = k + 1

    print('error =', e)
    print('iterations =', k)
    # if (round(e[0][0], 3) != 0 or  round(e[1][0], 3) != 0 or round(e[2][0], 3) != 0):
    print('The difference between IK position and desired position is \n', p_des - p_theta_n)
    return theta_n


def get_point_to_point_motion(init_pos, final_pos, T, instantaneous_time='None'):
    '''Returns trajectory of the motion between initial and final position
      with zero initial and final velocity and acceleration (resolution = 1 millisecond).
      If 'instantaneous_time' is provided, returns two nx1 arrays representing
      position and velocity in the next millisecond. (n = no. of rows in init_pos, final_pos)
    '''
    if instantaneous_time == 'None':
        position = np.zeros(((np.size(init_pos, 0)), 1))
        velocity = position
        for t in range(0, T + 1):
            t_ratio = t / T
            pos = init_pos + (10 * ((t_ratio) ** 3) - 15 * ((t_ratio) ** 4) + 6 * ((t_ratio) ** 5)) * (
                    final_pos - init_pos)
            vel = (30 * (t ** 2) / (T ** 3) - 15 * 4 * (t ** 3) / (T ** 4) + 6 * 5 * (t ** 4) / (T ** 5)) * (
                    final_pos - init_pos)
            position = np.append(position, pos.reshape(3, 1), axis=1)
            velocity = np.append(velocity, vel.reshape(3, 1), axis=1)

        position = np.delete(position, 0, axis=1)
        velocity = np.delete(velocity, 0, axis=1)
        return position, velocity  # Returns two 7x5001 arrays mapping the entire motion
    else:
        t = instantaneous_time
        t_ratio = t / T
        next_pos = init_pos + (10 * (t_ratio ** 3) - 15 * (t_ratio ** 4) + 6 * (t_ratio ** 5)) * (
                final_pos - init_pos)
        next_vel = (30 * (t ** 2) / (T ** 3) - 15 * 4 * (t ** 3) / (T ** 4) + 6 * 5 * (t ** 4) / (T ** 5)) * (
                final_pos - init_pos)
        return next_pos, next_vel


def get_local_motion(init_pos, init_vel, final_pos, final_vel, T, instantaneous_time='None'):
    '''Function generates a cubic trajectory with given parameters
    '''
    delta_pos = final_pos - init_pos
    r_dim, c_dim = init_pos.shape
    delta_vel = final_vel - init_vel
    a_0 = init_pos
    a_1 = init_vel
    a_3 = (1 / (T ** 2)) * (final_vel + init_vel - (2 * delta_pos / T))
    a_2 = 0.5 * (delta_vel / T - 3 * a_3 * T)

    if instantaneous_time == 'None':
        position = np.zeros(((np.size(init_pos, 0)), 1))
        velocity = position
        for t in range(0, T + 1):
            pos = a_0 + a_1 * t + a_2 * (t ** 2) + a_3 * (t ** 3)
            vel = a_1 + 2 * a_2 * t + 3 * a_3 * (t ** 2)
            position = np.append(position, pos.reshape(r_dim, 1), axis=1)
            velocity = np.append(velocity, vel.reshape(r_dim, 1), axis=1)

        position = np.delete(position, 0, axis=1)
        velocity = np.delete(velocity, 0, axis=1)
        return position, velocity  # Returns two 7x5001 arrays mapping the entire motion
    else:
        t = instantaneous_time

        # Calculating trajectories
        desired_curr_pos = a_0 + a_1 * t + a_2 * (t ** 2) + a_3 * (t ** 3)
        desired_curr_vel = a_1 + 2 * a_2 * t + 3 * a_3 * (t ** 2)

        return desired_curr_pos, desired_curr_vel
