from msvcrt import getch
import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy
import math

class TwoLinkRobot:

	def __init__(self):
		# Parameters
		self.M1 = 0.153	  # kg       link1 mass + servo2
		self.M2 = 0.245   # kg       link2 mass + extra load
		self.l1 = 0.131   # meter    length from joint1 to CG1
		self.l2 = 0.0789  # meter    length from joint2 to CG2
		self.L1 = 0.192   # meter    length from joint1 to joint2
		self.L2 = 0.156   # meter    length from joint2 to end effector
		self.g = 9.81     # gravity
		self.I1 = self.M1*self.l1**2  #Inertia of link1
		self.I2 = self.M2*self.l2**2  #Inertia of link2
		self.b1 = 0.1
		self.b2 = self.b1

		####################################################### Set Servo Configuration #############################################################
		self.ADDR_PRO_MODEL_NUMBER       		 = 0
		self.ADDR_PRO_DRIVE_MODE         		 = 10
		self.ADDR_PRO_OPERATING_MODE     		 = 11

		self.ADDR_PRO_CURRENT_LIMIT      		 = 38
		self.ADDR_PRO_ACCELERATION_LIMIT 		 = 40
		self.ADDR_PRO_VELOCITY_LIMIT     		 = 44

		self.ADDR_PRO_TORQUE_ENABLE      		 = 64               # Control table address is different in Dynamixel model

		self.ADDR_PRO_POSITION_D_GAIN    		 = 80
		self.ADDR_PRO_POSITION_I_GAIN    		 = 82
		self.ADDR_PRO_POSITION_P_GAIN    		 = 84

		self.ADDR_PRO_FEEDFORWARD_2nd_GAIN		 = 88
		self.ADDR_PRO_FEEDFORWARD_1st_GAIN 		 = 90

		self.ADDR_PRO_GOAL_CURRENT       		 = 102
		self.ADDR_PRO_GOAL_VELOCITY      		 = 104

		self.ADDR_PRO_PROFILE_ACCELERATION  	 = 108
		self.ADDR_PRO_PROFILE_VELOCITY   		 = 112

		self.ADDR_PRO_GOAL_POSITION      		 = 116
		self.ADDR_PRO_MOVING             		 = 122
		self.ADDR_PRO_MOVING_STATUS       		 = 123

		self.ADDR_PRO_PRESENT_CURRENT    		 = 126
		self.ADDR_PRO_PRESENT_VELOCITY   		 = 128 
		self.ADDR_PRO_PRESENT_POSITION   		 = 132


		# Data Byte Length
		self.LEN_PRO_GOAL_POSITION       	     = 4
		self.LEN_PRO_PRESENT_POSITION            = 4
		self.LEN_PRO_GOAL_CURRENT				 = 2
		self.LEN_PRO_PRESENT_CURRENT             = 2
		self.LEN_PRO_POS_VEL                     = 8
		self.LEN_PRO_POS_TIME                    = 12
		self.LEN_PRO_CUR_VEL_POS                 = 10
		self.LEN_PRO_POS_TORQUE                  = 18	

		# Operating Mode Number
		self.CURRENT_CONTROL                     = 0
		self.POSITION_CONTROL                    = 3 # Default
		self.CURRENT_BASED_POSITION_CONTROL      = 5

		self.TIME_BASED                          = 4
		self.VELOCITY_BASED                      = 0

		# Protocol version
		self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

		# ID
		self.DXL1_ID                      = 1                          
		self.DXL2_ID                      = 2                             
		self.DXL3_ID                      = 3                            
		self.DXL4_ID                      = 4
		self.DXL5_ID                      = 5
		self.DXL6_ID                      = 6
		self.DXL7_ID                      = 7

		self.BAUDRATE                    = 57600             # Dynamixel default self.BAUDRATE : 57600
		self.DEVICENAME                  = 'COM3'    # Check which port is being used on your controller
		                                                	 # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		# Open port
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()

		# Set port BAUDRATE
		if self.portHandler.setBaudRate(self.BAUDRATE):
			print("Succeeded to change the BAUDRATE")
		else:
			print("Failed to change the BAUDRATE")
			print("Press any key to terminate...")
			getch()
			quit()

		self.groupSyncWritePositionTorque = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_CURRENT, self.LEN_PRO_POS_TORQUE)

		self.groupSyncWriteDriveTorque = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_CURRENT, self.LEN_PRO_GOAL_CURRENT)

		#self.groupSyncReadVelocityPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)
		self.groupBulkReadVelocityPosition = GroupBulkRead(self.portHandler, self.packetHandler)

		self.groupBulkReadTorqueVelocityPosition = GroupBulkRead(self.portHandler, self.packetHandler)

		# Initialize GroupSyncRead instace for Present Position
		self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)


		# Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL1_ID)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL1_ID)
			quit()

		# Add parameter storage for Dynamixel#2 present position value
		dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL2_ID)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL2_ID)
			quit()

		# Add parameter storage for Dynamixel#1 present torque/velocity/position
		dxl_addparam_result = self.groupBulkReadTorqueVelocityPosition.addParam(self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_CUR_VEL_POS)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkRead addparam failed" % self.DXL1_ID)
			quit()

		# Add parameter storage for Dynamixel#2 present torque/velocity/position
		dxl_addparam_result = self.groupBulkReadTorqueVelocityPosition.addParam(self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_CUR_VEL_POS)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkRead addparam failed" % self.DXL2_ID)
			quit()

		# Add parameter storage for Dynamixel#1 present velocity/position
		dxl_addparam_result = self.groupBulkReadVelocityPosition.addParam(self.DXL1_ID, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkRead addparam failed" % self.DXL1_ID)
			quit()

		# Add parameter storage for Dynamixel#2 present velocity/position
		dxl_addparam_result = self.groupBulkReadVelocityPosition.addParam(self.DXL2_ID, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupBulkRead addparam failed" % self.DXL2_ID)
			quit()


		############################# Set Up the servo ##################################

		# Current based position control is also work, but not reaction force is not smooth as Current control
		self.DriveMode(self.TIME_BASED)
		#self.SetPID(1,1200,3000,1000) # PID gain for servo position loop control (1,900,0,1000)
		#self.SetPID(2,1200,3000,1000)

		
		self.InitTor1 = self.M1*self.l1*self.g + self.M2*self.L1*self.g + self.M2*self.l2*self.g   # initial torque at 0deg 
		self.InitTor2 = self.M2*self.l2*self.g                                                     # initial torque at 0deg 

		self.maxTorque = 7.0     # According to servo spec
		self.SetCurrentLimit(1,1000)
		self.SetCurrentLimit(2,1000)                                                

	def map(self,val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def SetOperatingMode(self,ID,MODE):

		self.TorqueOff()

		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE, MODE)

		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE)
		if present_mode == 0:
			# Current (Torque) Control Mode
			print("Now Operating Mode is Torque Control")
		elif present_mode == 3:
			# Position Control Mode
			print("Now Operating Mode is Position Control")
		elif present_mode == 5:
			# Current-based Position Control Mode
			print("Now Operating Mode is Current-based Position Control")
		else:
			print("In other Mode that didn't set!")

	def GetOperatingMode(self,ID):
		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE)

		return present_mode

	def SetCurrentLimit(self,ID,Lim_Current):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_CURRENT_LIMIT, Lim_Current)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		print("Set Current Limit of ID" + str(ID) + ' : ' + str(Lim_Current))

	def RunServo(self,ID,inputDeg1):

		pos1 = inputDeg1+90.0 # add some offset for robot kinematics
		servo_com1 = self.map(pos1,0.0,360.0,0.0,4095.0)
		dxl1_goal_position = int(servo_com1)

		dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
		if dxl_comm_result1 != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
		elif dxl_error1 != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error1))

	def RunPositionTorque(self,inputDeg1,Torque1,inputDeg2,Torque2):

		inputDeg1 = inputDeg1 + 90.0
		servo_ang1 = self.map(inputDeg1, 0.0, 360.0, 0, 4095)
		dxl1_goal_position = int(servo_ang1)

		inputDeg2 = inputDeg2 + 180.0
		servo_ang2 = self.map(inputDeg2, 0.0, 360.0, 0, 4095)
		dxl2_goal_position = int(servo_ang2)

		t1 = 0
		t3 = 0
		goal_vel =  1023  # default as velocity limited
		drive_current1 = ( (Torque1/2.1)*(240/413))  # A
		com_current1 = (drive_current1*1000)/2.69           # com value
		com_current1 = numpy.int16(com_current1)

		drive_current2 = ( (Torque2/2.1)*(240/413))  # A
		com_current2 = (drive_current2*1000)/2.69           # com value
		com_current2 = numpy.int16(com_current2)

		#print("com_curent", com_current)
		position_torque1 = [DXL_LOBYTE(com_current1), DXL_HIBYTE(com_current1),
							DXL_LOBYTE(DXL_LOWORD(goal_vel)), 
							DXL_HIBYTE(DXL_LOWORD(goal_vel)),
							DXL_LOBYTE(DXL_HIWORD(goal_vel)), 
							DXL_HIBYTE(DXL_HIWORD(goal_vel)),   
							DXL_LOBYTE(DXL_LOWORD(t1)), 
							DXL_HIBYTE(DXL_LOWORD(t1)),
							DXL_LOBYTE(DXL_HIWORD(t1)), 
							DXL_HIBYTE(DXL_HIWORD(t1)),
							DXL_LOBYTE(DXL_LOWORD(t3)), 
							DXL_HIBYTE(DXL_LOWORD(t3)),
							DXL_LOBYTE(DXL_HIWORD(t3)), 
							DXL_HIBYTE(DXL_HIWORD(t3)),
							DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), 
							DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)),
							DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), 
							DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]

		position_torque2 = [DXL_LOBYTE(com_current2), DXL_HIBYTE(com_current2),
							DXL_LOBYTE(DXL_LOWORD(goal_vel)), 
							DXL_HIBYTE(DXL_LOWORD(goal_vel)),
							DXL_LOBYTE(DXL_HIWORD(goal_vel)), 
							DXL_HIBYTE(DXL_HIWORD(goal_vel)),   
							DXL_LOBYTE(DXL_LOWORD(t1)), 
							DXL_HIBYTE(DXL_LOWORD(t1)),
							DXL_LOBYTE(DXL_HIWORD(t1)), 
							DXL_HIBYTE(DXL_HIWORD(t1)),
							DXL_LOBYTE(DXL_LOWORD(t3)), 
							DXL_HIBYTE(DXL_LOWORD(t3)),
							DXL_LOBYTE(DXL_HIWORD(t3)), 
							DXL_HIBYTE(DXL_HIWORD(t3)),
							DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), 
							DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
							DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), 
							DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]

		# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionTorque.addParam(self.DXL1_ID, position_torque1)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL1_ID)
			quit()

		# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionTorque.addParam(self.DXL2_ID, position_torque2)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL2_ID)
			quit()

		# Syncwrite goal position
		dxl_comm_result = self.groupSyncWritePositionTorque.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWritePositionTorque.clearParam()


	def ReadAngle(self):

		# Syncread present position
		dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		print("COMM_SUCCESS",COMM_SUCCESS)

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL1_ID)
			quit()

		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL2_ID)
			quit()

		# Get Dynamixel#1 present position value
		dxl_present_position1 = self.groupSyncReadPosition.getData(self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		# Get Dynamixel#2 present position value
		dxl_present_position2 = self.groupSyncReadPosition.getData(self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)

		# convert uint32 to int32, numpy,int32() doesn't work in case of number is too big
		if dxl_present_position1 > 2147483648:
			dxl_present_position1 = dxl_present_position1 - 4294967296

		# convert uint32 to int32, numpy,int32() doesn't work in case of number is too big
		if dxl_present_position2 > 2147483648:
			dxl_present_position2 = dxl_present_position2 - 4294967296

		Angle1 = self.map(dxl_present_position1, 0, 4095, 0.0, 360.0)
		# Check if deg is 360 or not, if positive and over than 360, deg would be reseted and count from 0 again
		if Angle1 >= 0:
			Angle1 = Angle1%360
		elif Angle1 < 0:
			Angle1 = Angle1%-360
		#print("Raw Pos",dxl_present_position1)

		Angle2 = self.map(dxl_present_position2, 0, 4095, 0.0, 360.0)
		if Angle2 >= 0:
			Angle2 = Angle2%360
		elif Angle2 < 0:
			Angle2 = Angle2%-360
		#print("Raw Pos",dxl_present_position1)

		Angle1 = Angle1 - 90
		Angle2 = Angle2 - 180

		return Angle1, Angle2

	def ReadCurrent(self,ID):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		
		# Check what is the model of this servo
		ServoModel = ReadModelNumber(1)
		if ServoModel == 321:
			SetUnit = UnitMX106
		elif ServoModel == 1020:
			SetUnit = UnitXM430
		elif ServoModel == 1120:
			SetUnit = UnitXM540
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" %self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" %self.packetHandler.getRxPacketError(dxl_error))
		
		com_signed_value = numpy.int16(dxl_present_current)
		if str(ID) == "2":
			self.current_unit = 3.36
		elif str(ID) == "3":
			self.current_unit= 2.69
		
		current = com_signed_value*2.69
		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current" + str(ID) + " : [ComValue_signed]: " + str(com_signed_value))
		print("Present current" + str(ID) + " : [mA] : " + str(current))

		return current

	def IsMoving1(self):
		Moving1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING)
		return Moving1

	def TorqueOn(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

	def TorqueOff(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

	def SetPID(self,ID,set_P_Gain,set_I_Gain,set_D_Gain):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain " + str(ID) + " : " + str(position_P_gain))
		print("Position I Gain " + str(ID) + " : " + str(position_I_gain))
		print("Position D Gain " + str(ID) + " : " + str(position_D_gain))
		print("------------------------------")


	def DriveMode(self,Base):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_DRIVE_MODE, Base)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_DRIVE_MODE, Base)

		DriveMode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_DRIVE_MODE)
		print("Drive Mode", DriveMode)


	def ReadVelPos(self):
		# Syncread present position
		dxl_comm_result = self.groupBulkReadVelocityPosition.txRxPacket()

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			print("TxRx error")

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = self.groupBulkReadVelocityPosition.isAvailable(self.DXL1_ID, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupBulkRead getdata failed" % self.DXL1_ID)
			print("isAvailable error")
			quit()
		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = self.groupBulkReadVelocityPosition.isAvailable(self.DXL2_ID, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupBulkRead getdata failed" % self.DXL2_ID)
			print("isAvailable error")
			quit()

		# Get Dynamixel#1 present position value
		present_velocity1, present_position1 = self.groupBulkReadVelocityPosition.getData(self.DXL1_ID, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)
		# Get Dynamixel#2 present position value
		present_velocity2, present_position2 = self.groupBulkReadVelocityPosition.getData(self.DXL2_ID, self.ADDR_PRO_PRESENT_VELOCITY, self.LEN_PRO_POS_VEL)

		# Clear syncread parameter storage
		## add this line make the code unable to read.... still dont understand
		#self.groupBulkReadVelocityPosition.clearParam()

		# Velocity
		# convert uint32 to int32, numpy,int32() doesn't work in case of number is too big
		if present_velocity1 > 2147483648:
			present_velocity1 = present_velocity1 - 4294967296
		#print("raw vel",dxl_present_velocity)
		if present_velocity2 > 2147483648:
			present_velocity2 = present_velocity2 - 4294967296
		#print("raw vel",dxl_present_velocity)
		rpm1 = present_velocity1*0.229
		vel1 = rpm1*(math.pi/60.0) # rad/s
		rpm2 = present_velocity2*0.229
		vel2 = rpm2*(math.pi/60.0) # rad/s


		# Position
		#print("raw pos",dxl_present_position)
		# convert uint32 to int32, numpy,int32() doesn't work in case of number is too big
		if present_position1 > 2147483648:
			present_position1 = present_position1 - 4294967296
		deg1 = self.map(present_position1,0,4095,0.0,360.0)
		# Check if deg is 360 or not, if positive and over than 360, deg would be reseted and count from 0 again
		if deg1 >= 0:
			deg1 = deg1%360
		elif deg1 < 0:
			deg1 = deg1%-360

		if present_position2 > 2147483648:
			present_position2 = present_position2 - 4294967296
		deg2 = self.map(present_position2,0,4095,0.0,360.0)
		# Check if deg is 360 or not, if positive and over than 360, deg would be reseted and count from 0 again
		if deg2 >= 0:
			deg2 = deg2%360
		elif deg2 < 0:
			deg2 = deg2%-360

		deg1 = deg1 - 90												# -90 for robot offset
		rad1 = deg1*math.pi/180.0     						 		    # rad

		deg2 = deg2 - 180												# -180 for robot offset
		rad2 = deg2*math.pi/180.0     						 		    # rad

		return vel1, rad1, vel2, rad2

	def ReadTorVelPos(self):
		# Syncread present position
		dxl_comm_result = self.groupBulkReadTorqueVelocityPosition.txRxPacket()

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			print("TxRx error")

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = self.groupBulkReadTorqueVelocityPosition.isAvailable(self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_CUR_VEL_POS)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupBulkRead getdata failed" % self.DXL1_ID)
			print("isAvailable error")
			quit()
		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = self.groupBulkReadTorqueVelocityPosition.isAvailable(self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_CUR_VEL_POS)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupBulkRead getdata failed" % self.DXL2_ID)
			print("isAvailable error")
			quit()

		# Get Dynamixel#1 present position value
		present_current1, present_velocity1, present_position1 = self.groupBulkReadTorqueVelocityPosition.getData(self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_CUR_VEL_POS)
		# Get Dynamixel#2 present position value
		present_current2, present_velocity2, present_position2 = self.groupBulkReadTorqueVelocityPosition.getData(self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_CUR_VEL_POS)

		# Clear syncread parameter storage
		## add this line make the code unable to read.... still dont understand
		#self.groupBulkReadVelocityPosition.clearParam()

		# Torque
		com_signed_value1 = numpy.int16(present_current1)
		#print("com_sign",com_signed_value)
		current1 = com_signed_value1*2.69                        #mA
		tau1 = ((current1/1000)*1.720833333)*2.1                 #N.m    multiplied by 2.1 for making correction with real load

		com_signed_value2 = numpy.int16(present_current2)
		#print("com_sign",com_signed_value)
		current2 = com_signed_value2*2.69                        #mA
		tau2 = ((current2/1000)*1.720833333)*2.1                 #N.m    multiplied by 2.1 for making correction with real load

		# Velocity
		# convert uint32 to int32, numpy,int32() doesn't work in case of number is too big
		if present_velocity1 > 2147483648:
			present_velocity1 = present_velocity1 - 4294967296
		#print("raw vel",dxl_present_velocity)
		if present_velocity2 > 2147483648:
			present_velocity2 = present_velocity2 - 4294967296
		#print("raw vel",dxl_present_velocity)
		rpm1 = present_velocity1*0.229
		vel1 = rpm1*(math.pi/60.0) # rad/s
		rpm2 = present_velocity2*0.229
		vel2 = rpm2*(math.pi/60.0) # rad/s


		# Position
		#print("raw pos",dxl_present_position)
		# convert uint32 to int32, numpy,int32() doesn't work in case of number is too big
		if present_position1 > 2147483648:
			present_position1 = present_position1 - 4294967296
		deg1 = self.map(present_position1,0,4095,0.0,360.0)
		# Check if deg is 360 or not, if positive and over than 360, deg would be reseted and count from 0 again
		if deg1 >= 0:
			deg1 = deg1%360
		elif deg1 < 0:
			deg1 = deg1%-360

		if present_position2 > 2147483648:
			present_position2 = present_position2 - 4294967296
		deg2 = self.map(present_position2,0,4095,0.0,360.0)
		# Check if deg is 360 or not, if positive and over than 360, deg would be reseted and count from 0 again
		if deg2 >= 0:
			deg2 = deg2%360
		elif deg2 < 0:
			deg2 = deg2%-360

		deg1 = deg1 - 90												# -90 for robot offset
		rad1 = deg1*math.pi/180.0     						 		    # rad

		deg2 = deg2 - 180												# -180 for robot offset
		rad2 = deg2*math.pi/180.0     						 		    # rad

		return tau1, vel1, rad1, tau2, vel2, rad2

	def DriveTorque(self,Torque1,Torque2):
		drive_current1 = ( (Torque1/2.1)*(240/413))         # A   remove +0.09 out (y intercept point)
		com_current1 = (drive_current1*1000)/2.69           # com value
		com_current1 = numpy.int16(com_current1)
		'''
		if abs(com_current1) < 31:
			com_current1 = 0
		'''
		drive_current2 = ( (Torque2/2.1)*(240/413))         # A   remove +0.09 out (y intercept point)
		com_current2 = (drive_current2*1000)/2.69           # com value
		com_current2 = numpy.int16(com_current2)
		'''
		if abs(com_current2) < 31:
			com_current2 = 0
		'''
		param_com_current1 = [DXL_LOBYTE(com_current1), DXL_HIBYTE(com_current1)]
		param_com_current2 = [DXL_LOBYTE(com_current2), DXL_HIBYTE(com_current2)]

		# Add Dynamixel#1 goal current value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWriteDriveTorque.addParam(self.DXL1_ID, param_com_current1)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL1_ID)
			quit()

		# Add Dynamixel#2 goal current value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWriteDriveTorque.addParam(self.DXL2_ID, param_com_current2)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL2_ID)
			quit()

		# Syncwrite goal position
		dxl_comm_result = self.groupSyncWriteDriveTorque.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWriteDriveTorque.clearParam()

		#print("Drive Torque", Torque)
		#print("com current",com_current)

	def ReadTorque(self,ID):
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" %self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" %self.packetHandler.getRxPacketError(dxl_error))

		com_signed_value = numpy.int16(dxl_present_current)
		#print("com_sign",com_signed_value)
		current = com_signed_value*2.69                        #mA
		tau = (( (current/1000) - 0.09)*1.720833333)*2.1      #N.m    multiplied by 2.1 for making correction with real load

		return tau
