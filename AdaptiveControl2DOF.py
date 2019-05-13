import time
from TwoLinkRobotClass import *
#from PygameLogicool import *

import numpy
import math
import matplotlib.pyplot as plt
from matplotlib import style

#joy = JoyStick()

twolink = TwoLinkRobot()

#twolink.SetOperatingMode(1,twolink.CURRENT_CONTROL)
#twolink.SetOperatingMode(2,twolink.CURRENT_CONTROL)

twolink.SetOperatingMode(1,twolink.CURRENT_BASED_POSITION_CONTROL)
twolink.SetOperatingMode(2,twolink.CURRENT_BASED_POSITION_CONTROL)

## This gain are set for position loop control, so to able to make the robot compensate the error, the gain value should be high enough for not letting position control affect a torque control loop
twolink.SetPID(1,2000,500,6000) 
twolink.SetPID(2,2000,500,6000)

twolink.TorqueOn()

twolink.RunPositionTorque(0,twolink.InitTor1,0,twolink.InitTor2)
#twolink.DriveTorque(twolink.InitTor1, twolink.InitTor2)

'''
while True:
	startTime = time.time()
	Tau1, Vel1, Pos1, Tau2, Vel2, Pos2 = twolink.ReadTorVelPos()
	print("Tau1", Tau1)
	print("Vel1", Vel1)
	print("Pos1", Pos1)
	print("Tau2", Tau2)
	print("Vel2", Vel2)
	print("Pos2", Pos2)
	#time.sleep(0.1)
	print("period", (time.time() - startTime))
	print("---------------------------------")
'''

# Parameters
m1 = twolink.M1
g = twolink.g
L1 = twolink.L1
m2 = twolink.M2
L2 = twolink.L2
l1 = twolink.l1
l2 = twolink.l2
I1 = twolink.I1
I2 = twolink.I2
maxTorque = twolink.maxTorque

PI = 3.141592654
deg2rad = 0.01745329252   # x pi/180

time.sleep(2)

theta1_s = 0
theta1_f = 60*math.pi/180  # 45deg

theta2_s = 0
theta2_f = -60*math.pi/180  # 30deg

ts = 0
tf = 2			# Finish time should not be over than 3 second, because it will cause the system move too slow and jerk
				# and should not lower than 1 second, it will cause the robot to overshoot

samplingTime = 0.02  # 50Hz due to reading and writing speed limit of Dynamixel   (In Device Manage, COM PORT, usb latency speed changed to 1ms)
totalPoint = tf/samplingTime
t = 0 # Time counter for function

# Controller Gain
kv = 0.2
ka = 0.1
gam1 = 0.6
gam2 = 0.6

Kp1 = 1   
Kv1 = kv
Ka1 = ka

Kp2 = 1   
Kv2 = kv
Ka2 = ka

init_m1 = 0.25
init_m2 = 0.25 

####################################### Generate Joint Trajectory #############################################
f = []
the1_d = []
vel1_d = []
acc1_d = []
the2_d = []
vel2_d = []
acc2_d = []
Y11 = []
Y12 = []
Y21 = []
Y22 = []
Time = []  # for plot

for i in range(0,int(totalPoint)):

	func = 6*(t/tf)**5 - 15*(t/tf)**4 + 10*(t/tf)**3
	the1 = theta1_s + (theta1_f - theta1_s)*func
	the1_d.append(the1)
	the2 = theta2_s + (theta2_f - theta2_s)*func
	the2_d.append(the2)

	if i == 0:
		vel1_d.append(0)
		acc1_d.append(0)
		vel2_d.append(0)
		acc2_d.append(0)
		Y11.append(0)
		Y12.append(0)
		Y21.append(0)
		Y22.append(0)
	else:
		vel1 = (the1_d[i] - the1_d[i-1])/samplingTime
		vel1_d.append(vel1)
		acc1 = (vel1_d[i] - vel1_d[i-1])/samplingTime
		acc1_d.append(acc1)

		vel2 = (the2_d[i] - the2_d[i-1])/samplingTime
		vel2_d.append(vel2)
		acc2 = (vel2_d[i] - vel2_d[i-1])/samplingTime
		acc2_d.append(acc2)

		# Regression matrix
		y11 = 2*l1**2*acc1 + g*l1*math.cos(the1)
		y12 = (L1**2 + l2**2 + L1*l2*math.cos(the2))*acc1 + l2**2*acc1 + (l2**2 + L1*l2*math.cos(the2))*acc2 + l2**2*acc2 - L1*l2*math.sin(the2)*(vel1*vel2 + 0.5*vel2**2) + g*L1*math.cos(the1) + g*l2*math.cos(the1+the2)
		y21 = 0
		y22 = (l2**2 + L1*l2*math.cos(the2))*acc1 + l2**2*acc1 + 2*l2**2*acc2 + 0.5*L1*l2*math.sin(the2)*vel1**2 + g*l2*math.cos(the1+the2)

		Y11.append(y11)
		Y12.append(y12)
		Y21.append(y21)
		Y22.append(y22) 


	Time.append(t)
	t = t + samplingTime

'''
plt.figure(1)
plt.plot(Time,the_d,'r')
plt.plot(Time,vel_d,'g')
plt.plot(Time,acc_d,'b')
plt.grid()
plt.title("Joint trajectory")

plt.show()

print("Last theta",the_d[-1])
print("Last vel",vel_d[-1])
'''

##################################### Start Control Loop ########################################################
# For plotting
error1 = []
error1dot = []
GenTor1 = []
pre_the1 = []
com_the1 = []
KA1_plot = []
KP1_plot = []
KV1_plot = []
Y1_plot = []
measure_tau1 = []
tau_diff1 = []
holdTorque1 = []

error2 = []
error2dot = []
GenTor2 = []
pre_the2 = []
com_the2 = []
KA2_plot = []
KP2_plot = []
KV2_plot = []
Y2_plot = []
measure_tau2 = []
tau_diff2 = []
holdTorque2 = []

plotTime = []
runTime = 0


T = time.time()
previous_sum1 = 0
previous_sum2 = 0

# Holding torque at last position, if there is no load attach on end-effector, this should be lowest torque for robot
hold_tor1 = init_m1*g*l1*math.cos(the1_d[-1]) + init_m2*(g*L1*math.cos(the1_d[-1]) + g*l2*math.cos(the1_d[-1] + the2_d[-1]))
hold_tor2 = init_m2*g*l2*math.cos(the1_d[-1] + the2_d[-1])

# Trajectory Tracking
print("######################################### Tracking Trajectory #########################################")
for i in range(0,int(totalPoint)):

	T=T+samplingTime
	startTime = time.time()

	Tau1, Vel1, Pos1, Tau2, Vel2, Pos2 = twolink.ReadTorVelPos()

	theta1_error = the1_d[i] - Pos1
	vel1_error = vel1_d[i] - Vel1
	theta2_error = the2_d[i] - Pos2
	vel2_error = vel2_d[i] - Vel2



	## 0.01 rad is 0.57deg resolution
	## 0.015rad is 0.86deg resolution
	## 0.02rad is 1.15deg resolution
	## 0.025rad is 1.4deg resolution
	if abs(theta1_error) < 0.04:  
		theta1_error = 0
	else:
		print("pos error1", theta1_error)
	if abs(theta2_error) < 0.04:  
		theta2_error = 0
	else:
		print("pos error2", theta2_error)

	r1 = theta1_error + vel1_error
	r2 = theta2_error + vel2_error

	errorPow = theta1_error**2 + theta2_error**2

	KA_term1 = r1*Ka1*errorPow
	KA_term2 = r2*Ka2*errorPow

	KV_term1 = r1*Kv1
	KV_term2 = r2*Kv2

	KP_term1 = theta1_error*Kp1
	KP_term2 = theta2_error*Kp2

	m1dot_est = gam1*Y11[i]*r1 + gam1*Y21[i]*r2
	m2dot_est = gam2*Y12[i]*r1 + gam2*Y22[i]*r2

	# Integral to estimate mass
	if i == 0:
		m1_est = previous_sum1
		m2_est = previous_sum2
	else:
		m1_est = previous_sum1 + m1dot_est*samplingTime
		m2_est = previous_sum2 + m2dot_est*samplingTime

	
	if abs(r1) == 0 and abs(r2) == 0:
		m1_est = init_m1
		m2_est = init_m2
	

	previous_sum1 = m1_est
	previous_sum2 = m2_est

	Y1_term = Y11[i]*m1_est + Y12[i]*m2_est
	Y2_term = Y21[i]*m1_est + Y22[i]*m2_est

	Generate_Torque1 = Y1_term + KP_term1 + KV_term1 + KA_term1
	Generate_Torque2 = Y2_term + KP_term2 + KV_term2 + KA_term2

	## Checking if calculate torque is over the limit, so set it as maximum torque
	## for XM430, max. torque is around 3.0 to 4.0
	if abs(Generate_Torque1) > maxTorque:
		Generate_Torque1 = maxTorque*(Generate_Torque1/abs(Generate_Torque1))

	if abs(Generate_Torque2) > maxTorque:
		Generate_Torque2 = maxTorque*(Generate_Torque2/abs(Generate_Torque2))

	#print("Gen Torque", Generate_Torque)
	#twolink.DriveTorque(Generate_Torque1, Generate_Torque2)
	twolink.RunPositionTorque((the1_d[i]*180/math.pi),Generate_Torque1,(the2_d[i]*180/math.pi),Generate_Torque2)

	# Collecting data for plot, but if there is some spike (big error) just ignore
	if abs(theta1_error) < 2 and abs(theta2_error) < 2 :

		error1.append(theta1_error*180/math.pi)
		error1dot.append(vel1_error)
		GenTor1.append(Generate_Torque1)
		pre_the1.append(Pos1*180/math.pi)
		com_the1.append(the1_d[i]*180/math.pi)
		KA1_plot.append(KA_term1)
		KV1_plot.append(KV_term1)
		KP1_plot.append(KP_term1)
		Y1_plot.append(Y1_term)
		measure_tau1.append(Tau1)
		tau_diff1.append(abs(Tau1) - abs(hold_tor1))
		holdTorque1.append(hold_tor1)

		error2.append(theta2_error*180/math.pi)
		error2dot.append(vel2_error)
		GenTor2.append(Generate_Torque2)
		pre_the2.append(Pos2*180/math.pi)
		com_the2.append(the2_d[i]*180/math.pi)
		KA2_plot.append(KA_term2)
		KV2_plot.append(KV_term2)
		KP2_plot.append(KP_term2)
		Y2_plot.append(Y2_term)
		measure_tau2.append(Tau2)
		tau_diff2.append(abs(Tau2) - abs(hold_tor2))
		holdTorque2.append(hold_tor2)


		plotTime.append(runTime)

		#print("runTime", runTime)

	runTime = runTime + samplingTime
	time.sleep(max(0,T-time.time()))     #max is needed in Windows due to
										#sleep's behaviour with negative argument
	#print("Period %.7f" %(time.time() - startTime))


T = time.time()
#startTime = time.time()
# Stabilize at final point

print("######################################### Stabilize #############################################")
for j in range(0,2000):
	T=T+samplingTime
	startTime = time.time()

	Tau1, Vel1, Pos1, Tau2, Vel2, Pos2 = twolink.ReadTorVelPos()

	#joyInputTheta = (15*math.pi/180)*Ax0

	theta1_error = (the1_d[-1]) - Pos1
	vel1_error = 0 - Vel1
	theta2_error = (the2_d[-1]) - Pos2
	vel2_error = 0 - Vel2

	#print("Tau1", Tau1)
	#print("Tau2", Tau2)

	## 0.01 rad is 0.57deg resolution
	## 0.015rad is 0.86deg resolution
	## 0.02 rad is 1.15deg resolution
	## 0.025rad is 1.4deg resolution

	if abs(theta1_error) < 0.04:  
		theta1_error = 0
	else:
		print("pos error1", theta1_error)
	if abs(theta2_error) < 0.04:  
		theta2_error = 0
	else:
		print("pos error2", theta2_error)


	errorPow = theta1_error**2 + theta2_error**2

	r1 = theta1_error + vel1_error
	r2 = theta2_error + vel2_error

	KA_term1 = r1*Ka1*errorPow
	KA_term2 = r2*Ka2*errorPow

	KV_term1 = r1*Kv1
	KV_term2 = r2*Kv2

	KP_term1 = theta1_error*Kp1
	KP_term2 = theta2_error*Kp2

	m1dot_est = gam1*Y11[-1]*r1 + gam1*Y21[-1]*r2
	m2dot_est = gam2*Y12[-1]*r1 + gam2*Y22[-1]*r2

	#print("m1dot_est", m1dot_est)
	#print("m2dot_est", m2dot_est)

	m1_est = previous_sum1 + m1dot_est*samplingTime
	m2_est = previous_sum2 + m2dot_est*samplingTime

	#print("m1_est", m1_est)
	#print("m2_est", m2_est)

	
	Y1_term = Y11[-1]*m1_est + Y12[-1]*m2_est
	Y2_term = Y21[-1]*m1_est + Y22[-1]*m2_est

	#Generate_Torque1 = Y1_term + KP_term1 + KV_term1 + KA_term1
	#Generate_Torque2 = Y2_term + KP_term2 + KV_term2 + KA_term2
	
	
	if abs(r1) == 0 and abs(r2) == 0 and abs(Tau1) - abs(hold_tor1)  < 0.1 and abs(Tau2) - abs(hold_tor2) < 0.1:
		# No error and torque from measure and holding are quite close, it means there is nothing attach
		# let generate torque as holding torque
		Generate_Torque1 = hold_tor1
		Generate_Torque2 = hold_tor2
		
		# If there is nothing attach on, so reset mass estimation to known initial mass
		previous_sum1 = init_m1
		previous_sum2 = init_m2
	else:
		# There is some error OR much different of measure and holding, it means there is some load attach on
		# let use generate torque from control law
		Generate_Torque1 = Y1_term + KP_term1 + KV_term1 + KA_term1
		Generate_Torque2 = Y2_term + KP_term2 + KV_term2 + KA_term2

		previous_sum1 = m1_est
		previous_sum2 = m2_est
	
	
	## Checking if calculate torque is over the limit, so set it as maximum torque
	## for XM430, max. torque is around 3.0 to 4.0
	if abs(Generate_Torque1) > maxTorque:
		Generate_Torque1 = maxTorque*(Generate_Torque1/abs(Generate_Torque1))

	if abs(Generate_Torque2) > maxTorque:
		Generate_Torque2 = maxTorque*(Generate_Torque2/abs(Generate_Torque2))
	
	#twolink.DriveTorque(Generate_Torque1, Generate_Torque2)
	twolink.RunPositionTorque((the1_d[-1]*180/math.pi),Generate_Torque1,(the2_d[-1]*180/math.pi),Generate_Torque2)

	# Collecting data for plot, but if there is some spike (big error) just ignore
	if abs(theta1_error) < 2 and abs(theta2_error) < 2:

		error1.append(theta1_error*180/math.pi)
		error1dot.append(vel1_error)
		GenTor1.append(Generate_Torque1)
		pre_the1.append(Pos1*180/math.pi)
		com_the1.append(the1_d[-1]*180/math.pi)
		KA1_plot.append(KA_term1)
		KV1_plot.append(KV_term1)
		KP1_plot.append(KP_term1)
		Y1_plot.append(Y1_term)
		measure_tau1.append(Tau1)
		tau_diff1.append(abs(Tau1) - abs(hold_tor1))
		holdTorque1.append(hold_tor1)

		error2.append(theta2_error*180/math.pi)
		error2dot.append(vel2_error)
		GenTor2.append(Generate_Torque2)
		pre_the2.append(Pos2*180/math.pi)
		com_the2.append(the2_d[-1]*180/math.pi)
		KA2_plot.append(KA_term2)
		KV2_plot.append(KV_term2)
		KP2_plot.append(KP_term2)
		Y2_plot.append(Y2_term)
		measure_tau2.append(Tau2)
		tau_diff2.append(abs(Tau2) - abs(hold_tor2))
		holdTorque2.append(hold_tor2)
		plotTime.append(runTime)

	#print("runTime", runTime)

	runTime = runTime + samplingTime

	time.sleep(max(0,T-time.time()))     #max is needed in Windows due to
	                                     #sleep's behaviour with negative argument
	#print("Period %.7f" %(time.time() - startTime))
	#loopTime = time.time() - startTime
	#print(loopTime)


plt.figure(1)
#plt.rcParams.update({'font.size': 18})
plt.subplot(4, 1, 1)
plt.plot(plotTime,error1,'r')
plt.plot(plotTime,error2,'b')
plt.grid()
plt.title("position error [deg]")
plt.legend(('pos_error1','pos_error2'), loc='best')
plt.subplot(4, 1, 2)
plt.plot(plotTime,error1dot,'r')
plt.plot(plotTime,error2dot,'b')
plt.grid()
plt.title("velocity error [deg/s]")
plt.legend(('vel_error1','vel_error2'), loc='best')
plt.subplot(4, 1, 3)
plt.plot(plotTime,GenTor1,'r')
plt.plot(plotTime,GenTor2,'b')
plt.grid()
plt.title("generated torque [N.m]")
plt.legend(('gen_tor1','gen_tor2'), loc='best')
plt.subplot(4, 1, 4)
plt.plot(plotTime,pre_the1,'r')
plt.plot(plotTime,com_the1,'k--')
plt.plot(plotTime,pre_the2,'b')
plt.plot(plotTime,com_the2,'g--')
plt.grid()
plt.title("Angle [deg]")
plt.legend(('output angle1','command angle1','output angle2','command angle2'), loc='best')

plt.figure(2)
plt.subplot(4, 1, 1)
plt.plot(plotTime,Y1_plot,'r')
plt.plot(plotTime,Y2_plot,'b')
plt.grid()
plt.title("Y term")
plt.legend(('1','2'), loc='best')
plt.subplot(4, 1, 2)
plt.plot(plotTime,KP1_plot,'r')
plt.plot(plotTime,KP2_plot,'b')
plt.grid()
plt.title("Kp term")
plt.legend(('1','2'), loc='best')
plt.subplot(4, 1, 3)
plt.plot(plotTime,KV1_plot,'r')
plt.plot(plotTime,KV2_plot,'b')
plt.grid()
plt.title("Kv term")
plt.legend(('1','2'), loc='best')
plt.subplot(4, 1, 4)
plt.plot(plotTime,KA1_plot,'r')
plt.plot(plotTime,KA2_plot,'b')
plt.grid()
plt.title("Ka term")
plt.legend(('1','2'), loc='best')

plt.figure(3)
plt.subplot(2, 1, 1)
plt.plot(plotTime,measure_tau1,'r')
plt.plot(plotTime,measure_tau2,'b')
plt.plot(plotTime,holdTorque1,'k--')
plt.plot(plotTime,holdTorque2,'g--')
plt.grid()
plt.title("Measured Torque")
plt.legend(('measure_torque1','measure_torque2','hold_torque1', 'hold torque2'), loc='best')
plt.subplot(2, 1, 2)
plt.plot(plotTime,tau_diff1,'r')
plt.plot(plotTime,tau_diff2,'b')
plt.grid()
plt.title("Measure - Holding")
plt.legend(('torque_diff1','torque_diff2'), loc='best')


plt.show()

#totalTime = time.time() - startTime
#print(totalTime)  
