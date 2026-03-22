#!/usr/bin/env python3

from DynamixelSDK.python.src.dynamixel_sdk import *



class ControlMotors:
    def __init__(self, motor_ids, port):
        self.motor_ids = motor_ids

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0) #Protocol 2.0

        #Constants needed
        self.goal_position_address = 116
        self.present_position_address = 132
        self.torque_on_address = 64
        self.goal_position_length = 4
        self.present_position_length = 4

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
        for motor_id in motor_ids:
            communication_result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.torque_on_address, 1) 
            if communication_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(communication_result))
            elif error != 0:
                print("%s" % self.packetHandler.getRxPacketError(error))
            else:
                print("Dynamixel motor", motor_id ," is connected")


        #Enable torque / aka turn on movement
        for motor_id in self.motor_ids:
            communication_result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.torque_on_address, 1)
            if error != 0:
                print("enable torque failed for motor ", motor_id)


        #Add motors to bulk read
        for motor_id in motor_ids:
            addparam_result = self.groupBulkRead.addParam(motor_id, self.present_position_address, self.goal_position_length)
            if addparam_result != True:
                print("groupBulkRead addparam failed for ", motor_id)




    def set_position(self, target_positions):
        self.groupBulkWrite.clearParam()

        #Set target positon for motors
        for motor_id, target_position in zip(self.motor_ids, target_positions):
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(target_position)),
                DXL_HIBYTE(DXL_LOWORD(target_position)),
                DXL_LOBYTE(DXL_HIWORD(target_position)),
                DXL_HIBYTE(DXL_HIWORD(target_position)),
            ]

            dxl_addparam_result = self.groupBulkWrite.addParam(
                motor_id,
                self.goal_position_address,
                self.goal_position_length,
                param_goal_position
            )

        #Write in bulk
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))

        self.groupBulkWrite.clearParam()



    def get_position(self):
        #temp variable for storing positions
        positions = []

        #Read
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))
            return positions

        #Read all positions
        for motor_id in self.motor_ids:
            #Read position
            position = self.groupBulkRead.getData(
                motor_id,
                self.present_position_address,
                self.present_position_length
            )

            #append position to list/array
            positions.append(position)


        return positions



    def close(self):
        for motor_id in self.motor_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, self.torque_on_address, 0
            )
        self.portHandler.closePort()



# #Test script below
# port_aksel_pc = "/dev/ttyUSB0"
# motor_ids_test = [1, 2, 3, 4]
# motors = ControlMotors(motor_ids_test, port_aksel_pc)


# while True:
#     targets = []

#     for motor_id in motor_ids_test:
#         target_position = int(
#             input(f"Enter target position for motor {motor_id} (0 ~ 4095) or -1 to exit: ")
#         )

#         if target_position == -1:
#             motors.close()
#             exit()

#         if target_position < 0 or target_position > 4095:
#             print("Position must be between 0 and 4095.")
#             targets = []
#             break

#         targets.append(target_position)

#     if len(targets) != len(motor_ids_test):
#         continue

#     motors.set_position(targets)

#     while True:
#         positions = motors.get_position()

#         if len(positions) != len(motor_ids_test):
#             print("Failed to read all motor positions.")
#             continue

#         for motor_id, position in zip(motor_ids_test, positions):
#             print(f"Motor {motor_id} Position: {position}")

#         if all(abs(target - position) <= 10 for target, position in zip(targets, positions)):
#             print("All motors reached target.")
#             break