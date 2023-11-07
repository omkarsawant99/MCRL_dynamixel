from dynamixel_sdk import *
import time
import matplotlib.pyplot as plt

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_TORQUE_CONTROL_ENABLE = 70
ADDR_MX_GOAL_TORQUE        = 71

# Protocol version
PROTOCOL_VERSION = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID     = 1                 # Dynamixel ID : 1
BAUDRATE   = 57600             # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_VALUE  = 100
ENABLE  = 1                 # Value for enabling the torque
DISABLE = 0                 # Value for disabling the torque

class MotorController:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.open_port()
        self.set_baudrate()

    def open_port(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

    def set_baudrate(self):
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

    def enable_torque(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_CONTROL_ENABLE, ENABLE)
        self.check_comm_result(dxl_comm_result, dxl_error)

    def convert_nm_to_motor_val(self, val_nm, max_torque_nm):
        if not 0 <= abs(val_nm) <= 0.8*max_torque_nm:
            print(val_nm)
            raise ValueError("Torque exceeds maximum limit!!")
        
        motor_val = (1023*val_nm)/max_torque_nm

        if motor_val < 0:
            motor_val = 1024 + abs(motor_val)

        if not 0 <= motor_val <= 2047:
            print(motor_val)
            raise ValueError("Scale exceeds maximum limit!!")

        return int(motor_val)

    def set_torque(self, dxl_id, val):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_GOAL_TORQUE, val)
        self.check_comm_result(dxl_comm_result, dxl_error)

    def disable_torque(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_CONTROL_ENABLE, DISABLE)
        self.check_comm_result(dxl_comm_result, dxl_error)

    def check_comm_result(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Is it overload?")
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def close_port(self):
        self.portHandler.closePort()

'''
if __name__ == "__main__":
    motor_control = MotorController()

    motor_control.enable_torque(DXL_ID)

    tick = time.time()
    while 1:
        tock = time.time()
        #print(tock - tick)
        if tock - tick > 5:
            break
        
        tau = motor_control.convert_nm_to_motor_val(0.4125, 8.4)
        print(tau)
        motor_control.set_torque(DXL_ID, tau)

    motor_control.disable_torque(DXL_ID)

    motor_control.close_port()
'''