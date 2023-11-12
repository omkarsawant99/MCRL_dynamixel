import pinocchio as pin
import meshcat
import numpy as np
import time
import sys

from pinocchio.visualize import MeshcatVisualizer

class RobotEnv:
    def __init__(self, pin_robot):
        self.model = pin_robot.model
        self.data = pin_robot.data
        self.viz = pin.visualize.MeshcatVisualizer(pin_robot.model, pin_robot.collision_model, pin_robot.visual_model)

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


    def show_target(self, pos):
        self.viz.viewer['ball'].set_transform(meshcat.transformations.translation_matrix(pos))

    def show_positions(self, q):
        self.viz.display(q)

    def show_neutral_positions(self):
        q0 = pin.neutral(self.model)
        print(q0)
        self.viz.display(q0)

    def show_random_positions(self):
        q = np.random.sample([6])
        self.viz.display(q)

    def get_spatial_jacobian(self, q):
        '''J = [J-linear(3xnum_joint);
                J-angular(3xnum_joint)]'''
        J = pin.computeJointJacobians(self.model, self.data, q)
        temp = J[:3, :]
        J = np.delete(J, [0, 1, 2], axis=0)
        J = np.vstack((J, temp))

        return J

    def get_forward_kinematics(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        end_eff_id = self.model.getFrameId("joint_tip")
        end_eff = self.data.oMf[end_eff_id]
        T = np.hstack((end_eff.rotation, end_eff.translation.reshape(-1, 1)))
        T = np.vstack((T, np.array([[0, 0, 0, 1]])))
        return T

    def rnea(self, q, dq, ddq):
        return pin.pinocchio_pywrap.rnea(self.model, self.data, q, dq, ddq).reshape((6,1))

    def get_gravity(self, q):
        g = self.rnea(q, np.zeros((6,1)), np.zeros((6,1)))
        return g.reshape(-1, 1)


def simulate_robot(robot, planner, robot_controller):
    '''
    Simulates the robot with the given controller
    INPUT:  robot (Class RobotEnv) : Environment of the robot
            robot_controller       : Controller function (outputs torque)
    '''
    t = 0.
    dt = 0.001
    q = np.zeros((6,1))
    dq = np.zeros((6,1))
    robot.show_positions(q)
    t_visual = 0
    dt_visual = 0.01
    disturbance_active = False
    disturbance_force = np.zeros((6,1))
    while(True):
        # Activate disturbance at t = 1 second
        if t > 1.0 and t < 3.0 and not disturbance_active:
            #disturbance_force = np.random.normal(0, 0.5, (6,1))  # Random values with mean 0 and std dev 0.5
            disturbance_active = True

        # Deactivate disturbance at t = 3 seconds
        if t >= 3.0 and disturbance_active:
            disturbance_force = np.zeros((6,1))
            disturbance_active = False

        tau = robot_controller(robot, planner, t, q.reshape((6,1)), dq.reshape((6,1)))

        if disturbance_active:
            tau += disturbance_force
        
        ddq = pin.pinocchio_pywrap.aba(robot.model, robot.data, q, dq, tau)
        dq += dt * ddq.reshape((6,1))
        q += dt*dq
        if t_visual == 10:
            robot.show_positions(q)
            time.sleep(dt_visual)
            t_visual = 0
        t_visual += 1
        t += dt


def simulate_robot_real_time_pwm(robot, planner, robot_controller, MotorController):
    t = 0.
    dt = 0.001
    q = np.zeros((6,1))
    dq = np.zeros((6,1))
    robot.show_positions(q)
    MotorIDs = {1: 8.4, 2: 6.0, 6: 8.4}         # ID : Max torque at 12 V
    t_visual = 0
    dt_visual = 0.01

    
    MotorController.enable_torque(6)

    while(t <= 10):
        start_time = time.time()
        tau = robot_controller(robot, planner, t, q.reshape((6,1)), dq.reshape((6,1)))

        print("Controller says:", tau[1])
        val = MotorController.convert_nm_to_motor_val(tau[0], 8.4)
        MotorController.set_torque(6, val)
        
        ddq = pin.pinocchio_pywrap.aba(robot.model, robot.data, q, dq, tau)
        dq += dt * ddq.reshape((6,1))
        q += dt*dq

        if t_visual == 10:
            robot.show_positions(q)
            #MotorController.pwm_control(tau[1])
            time.sleep(dt_visual)
            t_visual = 0
        
        t_visual += 1
        t += dt
    
    for id in MotorIDs:
        MotorController.disable_torque(id)

    MotorController.close_port()

def simulate_robot_real_time(robot, planner, robot_controller, MotorController):
    t = 0.
    dt = 0.001
    q = np.zeros((6,1))
    dq = np.zeros((6,1))
    robot.show_positions(q)
    MotorIDs = {1: 8.4, 2: 8.4, 3: 6.0}         # ID : Max torque at 12 V
    t_visual = 0
    dt_visual = 0.01

    for id in MotorIDs:
        MotorController.enable_torque(id)

    while(t <= 10):
        start_time = time.time()
        tau = robot_controller(robot, planner, t, q.reshape((6,1)), dq.reshape((6,1)))
        print("Controller says:", tau[1])

        ddq = pin.pinocchio_pywrap.aba(robot.model, robot.data, q, dq, tau)
        dq += dt * ddq.reshape((6,1))
        q += dt*dq
        if t_visual == 10:
            robot.show_positions(q)
            time.sleep(dt_visual)
            for id, tu in zip(MotorIDs, tau):
                val = MotorController.convert_nm_to_motor_val(-tu, MotorIDs[id])
                print(val)
                MotorController.set_torque(id, val)
            t_visual = 0
        t_visual += 1
        t += dt
    
    for id in MotorIDs:
        MotorController.disable_torque(id)

    MotorController.close_port()