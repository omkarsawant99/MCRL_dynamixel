import numpy as np
import time
import sys
import matplotlib.pyplot as plt
import pinocchio as pin
import meshcat

from pinocchio.visualize import MeshcatVisualizer

class RobotEnv:
    def __init__(self, pin_robot, end_effector_name):
        self.model = pin_robot.model
        self.data = pin_robot.data
        self.viz = pin.visualize.MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)
        self.end_effector_name = end_effector_name

        # Constants to make things easier
        self.num_joints = pin_robot.model.nq
        self.zeros_nx1 = np.zeros((pin_robot.model.nq, 1))
        print(self.num_joints)

    def start_visualizer(self):
        try:
            self.viz.initViewer(open=True)
        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install Python meshcat"
            )
            print(err)
            sys.exit(0)

        # Load the robot in the viewer.
        self.viz.loadViewerModel()

        # Display a robot configuration.
        q0 = pin.neutral(self.model)
        self.viz.display(q0)
        self.viz.displayCollisions(True)
        self.viz.displayVisuals(False)

        # Display target
        self.viz.viewer['ball'].set_object(meshcat.geometry.Sphere(0.01),
                              meshcat.geometry.MeshLambertMaterial(
                             color=0xff22dd,
                             reflectivity=0.8))
        
        self.viz.viewer['end_eff'].set_object(meshcat.geometry.Sphere(0.01),
                              meshcat.geometry.MeshLambertMaterial(
                             color=0x22ff44,
                             reflectivity=0.8))


    def show_target(self, pos):
        self.viz.viewer['ball'].set_transform(meshcat.transformations.translation_matrix(pos))

    def show_end_effector(self, end_eff_pos):
        self.viz.viewer['end_eff'].set_transform(meshcat.transformations.translation_matrix(end_eff_pos))

    def show_positions(self, q):
        self.viz.display(q)

    def show_neutral_positions(self):
        q0 = pin.neutral(self.model)
        print(q0)
        self.viz.display(q0)

    def show_random_positions(self):
        q = np.random.sample((self.num_joints))
        self.viz.display(q)

    def get_spatial_jacobian(self, q):
        '''J = [J-linear(3xnum_joint);
                J-angular(3xnum_joint)]'''
        J = pin.computeJointJacobians(self.model, self.data, q)
        temp = J[:3, :]
        J = np.delete(J, [0, 1, 2], axis=0)
        J = np.vstack((J, temp))

        return J
    
    def get_end_effector_jacobian(self, q):
        J = pin.computeFrameJacobian(self.model, self.data, q, self.model.getFrameId(self.end_effector_name))
        temp = J[:3, :]
        J = np.delete(J, [0, 1, 2], axis=0)
        J = np.vstack((J, temp))

        return J

    def get_forward_kinematics(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        end_eff_id = self.model.getFrameId(self.end_effector_name)
        end_eff = self.data.oMf[end_eff_id]
        T = np.hstack((end_eff.rotation, end_eff.translation.reshape(-1, 1)))
        T = np.vstack((T, np.array([[0, 0, 0, 1]])))
        return T

    def rnea(self, q, dq, ddq):
        return pin.pinocchio_pywrap.rnea(self.model, self.data, q, dq, ddq).reshape(-1, 1)

    def get_gravity(self, q):
        g = self.rnea(q, self.zeros_nx1, self.zeros_nx1)
        return g.reshape(-1, 1)

    def get_coriolis(self, q, dq):
        C = self.rnea(q, dq, self.zeros_nx1) - self.get_gravity(q)
        return C

    def get_mass_matrix(self, q, dq):
        M = np.zeros((self.num_joints, self.num_joints))
        for i in range(self.num_joints):
            ddq = self.zeros_nx1
            ddq[i] = 1
            M[:, i] = pin.pinocchio_pywrap.rnea(self.model, self.data, q, self.zeros_nx1, ddq).reshape(-1) - self.get_gravity(q)
        return M


def simulate_robot(robot, planner, robot_controller, disturbance_all_joints=False, disturbance_end_effector=False):
    '''
    Simulates the robot with the given controller
    INPUT:  robot (Class RobotEnv) : Environment of the robot
            robot_controller       : Controller function (outputs torque)
    '''
    # Define constants
    t = 0.
    dt = 0.001
    q = np.zeros((robot.num_joints,1))
    dq = np.zeros((robot.num_joints,1))
    robot.show_positions(q)
    t_visual = 0
    dt_visual = 0.01
    disturbance_active = False
    disturbance_force = np.zeros((6, 1))

    # Initialize plot
    plot = LivePlot(x_label="Time(s)", y_label="Torques", title="Joint Torques over time", num_joints=robot.num_joints)

    while(t < 1.5):
        # Get end-effector (body jacobian) jacobian
        J_b = robot.get_end_effector_jacobian(q)
        
        # Retrieve torques
        tau = robot_controller(robot, planner, t, q.reshape((robot.num_joints, 1)), dq.reshape((robot.num_joints, 1)))

        # Activate disturbance at t seconds

        if t > 1.5 and t < 1.7 and not disturbance_active:
            if disturbance_all_joints:
                disturbance_force = np.random.normal(0, 0.5, (6, 1))            # Random values with mean 0 and std dev 0.5
            if disturbance_end_effector:
                disturbance_force = np.zeros((6, 1))
                disturbance_force[4, :] = -5                                  # Force in y-direction of the end-effector frame
            disturbance_active = True

        # Deactivate disturbance after t seconds
        if t >= 1.7 and disturbance_active:
            disturbance_active = False

        # Add disturbance
        disturbance_force_joint_space = J_b.T @ disturbance_force       # Converting that disturbance in joint space
        tau += disturbance_force_joint_space
        

        # Update visualizer
        ddq = pin.pinocchio_pywrap.aba(robot.model, robot.data, q, dq, tau)
        dq += dt * ddq.reshape((robot.num_joints, 1))
        q += dt*dq
        if t_visual == 10:
            robot.show_positions(q)
            plot.update_live_plot(t, tau)
            time.sleep(dt_visual)
            t_visual = 0
        t_visual += 1
        t += dt


def simulate_robot_real_time(robot, planner, robot_controller, MotorController, MotorIDs):
    t = 0.
    dt = 0.001
    q = np.zeros((robot.num_joints,1))
    dq = np.zeros((robot.num_joints,1))
    robot.show_positions(q)
    t_visual = 0
    dt_visual = 0.01

    for id in MotorIDs:
        MotorController.enable_torque(id)

    while(t <= 1):
        start_time = time.time()
        tau = robot_controller(robot, planner, t, q.reshape((-1,1)), dq.reshape((-1,1)))
        #print("Controller says:", tau)
        #print("---------------------")

        if t_visual == 10:
            robot.show_positions(q)
            time.sleep(dt_visual)
            for id, tu in zip(MotorIDs, tau):
                val = MotorController.convert_nm_to_motor_val(-tu.item(), MotorIDs[id])
                print(val)
                MotorController.set_torque(id, val)
            print("---------------")
            t_visual = 0
        t_visual += 1
        t += dt
    
    for id in MotorIDs:
        MotorController.disable_torque(id)

    MotorController.close_port()


def mimic_robot(robot, MotorController):
    t = 0.
    dt = 0.001
    q = np.zeros((robot.num_joints,1))
    robot.show_positions(q)
    MotorIDs = {1: 8.4, 2: 8.4, 3: 8.4}         # ID : Max torque at 12 V
    t_visual = 0
    dt_visual = 0.01

    for id in MotorIDs:
        MotorController.enable_torque(id)

    while(t <= 5):
        if t_visual == 10:
            for i, id in enumerate(MotorIDs):
                val = MotorController.read_pos(id)
                q[i] = val
                time.sleep(0.001)
            robot.show_positions(q*np.pi/180)
            time.sleep(dt_visual)
            t_visual = 0
        t_visual += 1
        t += dt
    
    for id in MotorIDs:
        MotorController.disable_torque(id)

    MotorController.close_port()

class LivePlot:
    def __init__(self, x_label, y_label, title, num_joints):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)
        self.ax.set_title(title)
        self.lines = [self.ax.plot([], [], label=f'Joint {i+1}')[0] for i in range(num_joints)]
        self.ax.legend()
        time.sleep(2)  # Short pause to allow the plot to render

    def update_live_plot(self, t, tau):
        # Update live plot
        for i, line in enumerate(self.lines):
            line.set_xdata(np.append(line.get_xdata(), t))
            line.set_ydata(np.append(line.get_ydata(), tau[i, 0]))

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
