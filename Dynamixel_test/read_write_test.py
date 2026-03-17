#!/usr/bin/env python3

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupBulkRead, GroupBulkWrite



class ControlMotors:
    def __init__(self, motor_id1, motor_id2, port):
        self.motor_id1 = motor_id1
        self.motor_id2 = motor_id2


        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0) #Protocol 2.0

        self.goal_position_address = 116
        self.present_position_address = 132
        self.torque_on_address = 64

        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)


        #Check port is open
        if self.portHandler.openPort():
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")
            exit()

        #Check baudrate is correct
        if self.portHandler.setBaudRate(57600): #Baudrate is set to 57600
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")
            exit()

        #Check that the connection is working
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id1, self.torque_on_address, 1) #64 is the address that enables torque (aka movement)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#", self.motor_id1 ," has been successfully connected")

        #Check that the connection is working
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id2, self.torque_on_address, 1) #64 is the address that enables torque (aka movement)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#", self.motor_id2 ," has been successfully connected")


    def set_position(self, target_position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, self.goal_position_address, target_position) #116 is the address for setting target position
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def get_position(self):
        present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.motor_id, self.present_position_address) #132 is the address for reading position
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return present_position

    def close(self):
        self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, 64, 0)
        self.portHandler.closePort()


port_aksel_pc = "/dev/ttyUSB0"
motor1 = ControlMotors(1, 2, port_aksel_pc)

while True:
    try:
        target_position = int(input("Enter target position (0 ~ 4095, -1 to exit): "))
    except ValueError:
        print("Please enter an integer.")
        continue

    if target_position == -1:
        break
    elif target_position < 0 or target_position > 4095:
        print("Position must be between 0 and 4095.")
        continue

    motor1.set_position(target_position)

    while True:
        present_position = motor1.get_position()
        print(f"Current Position: {present_position}")
        if abs(target_position - present_position) <= 10:
            break

motor1.close()