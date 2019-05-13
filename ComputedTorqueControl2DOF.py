import time
from TwoLinkRobotClass import *
#from PygameLogicool import *
import numpy
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#joy = JoyStick()

twolink = TwoLinkRobot()

twolink.SetOperatingMode(1,twolink.CURRENT_CONTROL)
twolink.SetOperatingMode(2,twolink.CURRENT_CONTROL)

#twolink.SetOperatingMode(1,twolink.CURRENT_BASED_POSITION_CONTROL)
#twolink.SetOperatingMode(2,twolink.CURRENT_BASED_POSITION_CONTROL)

twolink.TorqueOn()

twolink.DriveTorque(twolink.InitTor1,twolink.InitTor2) # N.m
#twolink.RunPositionTorque(0,twolink.InitTor1,0,twolink.InitTor2)
'''
while True:
    Vel1, Pos1, Vel2, Pos2 = twolink.ReadVelPos()
    print("Vel1", Vel1)
    print("Pos1", Pos1)
    print("Vel2", Vel2)
    print("Pos2", Pos2)
    time.sleep(0.1)
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

PI = 3.141592654
deg2rad = 0.01745329252   # x pi/180

time.sleep(2)

theta1_s = 0
theta1_f = math.pi/4  # 45deg

theta2_s = 0
theta2_f = math.pi/4  # 30deg

ts = 0
tf = 3
samplingTime = 0.02  # 50Hz due to reading and writing speed limit of Dynamixel   (In Device Manage, COM PORT, usb latency speed changed to 1ms)
totalPoint = tf/samplingTime
t = 0 # Time counter for function

# Controller Gain

Kp1 =400   
Kv1 = 2*math.sqrt(Kp1)
Ki1 = 400

Kp2 = 400   
Kv2 = 2*math.sqrt(Kp2)
Ki2 = 400

####################################### Generate Joint Trajectory #############################################
f = []
the1_d = []
vel1_d = []
acc1_d = []
the2_d = []
vel2_d = []
acc2_d = []
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
    else:
        vel1 = (the1_d[i] - the1_d[i-1])/samplingTime
        vel1_d.append(vel1)
        acc1 = (vel1_d[i] - vel1_d[i-1])/samplingTime
        acc1_d.append(acc1)

        vel2 = (the2_d[i] - the2_d[i-1])/samplingTime
        vel2_d.append(vel2)
        acc2 = (vel2_d[i] - vel2_d[i-1])/samplingTime
        acc2_d.append(acc2)

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
GenTor1 = []
pre_the1 = []
com_the1 = []
error2 = []
GenTor2 = []
pre_the2 = []
com_the2 = []
plotTime = []
runTime = 0



T = time.time()
previous_sum1 = 0
previous_sum2 = 0
# Trajectory Tracking
print("######################################### Tracking Trajectory #########################################")
for i in range(0,int(totalPoint)):

    T=T+samplingTime
    startTime = time.time()

    Vel1, Pos1, Vel2, Pos2 = twolink.ReadVelPos()

    theta1_error = the1_d[i] - Pos1
    vel1_error = vel1_d[i] - Vel1
    theta2_error = the2_d[i] - Pos2
    vel2_error = vel2_d[i] - Vel2

    ## 0.01 rad is 0.57deg resolution
    ## 0.015rad is 0.86deg resolution
    ## 0.02rad is 1.15deg resolution
    ## 0.025rad is 1.4deg resolution
    if abs(theta1_error) < 0.02:  
        theta1_error = 0
    if abs(theta2_error) < 0.02:  
        theta2_error = 0

    ## Error integral term
    if i == 0:
        error_integral1 = previous_sum1
        error_integral2 = previous_sum2
    elif i > 0:
        error_integral1 = previous_sum1 + samplingTime*theta1_error
        error_integral2 = previous_sum2 + samplingTime*theta2_error

    previous_sum1 = error_integral1
    previous_sum2 = error_integral2

    print("pos error1", theta1_error)
    print("pos error2", theta2_error)

    ## Mass matrix elements
    H11 = m1*l1**2 + m2*(L1**2 + l2**2 + L1*l2*math.cos(Pos2)) + I1 + I2
    H12 = m2*(l2**2 + L1*l2*math.cos(Pos2)) + I2
    H21 = m2*(l2**2 + L1*l2*math.cos(Pos2)) + I2
    H22 = m2*l2**2 + I2
    ## Collioris matrix elements
    V11 = -m2*L1*l2*math.sin(Pos2)*(Vel1*Vel2 + 0.5*Vel2**2)
    V21 = 0.5*m2*L1*l2*math.sin(Pos2)*Vel1**2
    ## Gravity matrix elements
    G11 = m1*g*l1*math.cos(Pos1) + m2*g*L1*math.cos(Pos1) + m2*g*l2*math.cos(Pos1 + Pos2)
    G21 = m2*g*l2*math.cos(Pos1+Pos2)

    Dynamic_error1 = acc1_d[i] + theta1_error*Kp1 + error_integral1*Ki1 + vel1_error*Kv1
    Dynamic_error2 = acc2_d[i] + theta2_error*Kp2 + error_integral2*Ki2 + vel2_error*Kv2

    Mass_Dynamic1 = H11*Dynamic_error1 + H12*Dynamic_error2
    Mass_Dynamic2 = H21*Dynamic_error1 + H22*Dynamic_error2

    Generate_Torque1 = Mass_Dynamic1 + V11 + G11
    Generate_Torque2 = Mass_Dynamic2 + V21 + G21

    #print("Gen Torque", Generate_Torque)
    twolink.DriveTorque(Generate_Torque1, Generate_Torque2)
    #twolink.RunPositionTorque((the1_d[i]*180/math.pi),Generate_Torque1,(the2_d[i]*180/math.pi),Generate_Torque2)
    
    # Collecting data for plot, but if there is some spike (big error) just ignore
    if abs(theta1_error) < 2 and abs(theta2_error) < 2:

        error1.append(theta1_error*180/math.pi)
        GenTor1.append(Generate_Torque1)
        pre_the1.append(Pos1*180/math.pi)
        com_the1.append(the1_d[i]*180/math.pi)

        error2.append(theta2_error*180/math.pi)
        GenTor2.append(Generate_Torque2)
        pre_the2.append(Pos2*180/math.pi)
        com_the2.append(the2_d[i]*180/math.pi)

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
for j in range(0,300):
    T=T+samplingTime
    startTime = time.time()

    Vel1, Pos1, Vel2, Pos2 = twolink.ReadVelPos()

    #joyInputTheta = (15*math.pi/180)*Ax0

    theta1_error = (the1_d[-1]) - Pos1
    vel1_error = 0 - Vel1
    theta2_error = (the2_d[-1]) - Pos2
    vel2_error = 0 - Vel2

    print("pos error1", theta1_error)
    print("pos error2", theta2_error)
    ## 0.01 rad is 0.57deg resolution
    ## 0.015rad is 0.86deg resolution
    ## 0.02 rad is 1.15deg resolution
    ## 0.025rad is 1.4deg resolution

    if abs(theta1_error) < 0.02:  
        theta1_error = 0
    if abs(theta2_error) < 0.02:  
        theta2_error = 0

    error_integral1 = previous_sum1 + samplingTime*theta1_error
    error_integral2 = previous_sum2 + samplingTime*theta2_error

    previous_sum1 = error_integral1
    previous_sum2 = error_integral2

    ## Mass matrix elements
    H11 = m1*l1**2 + m2*(L1**2 + l2**2 + L1*l2*math.cos(Pos2)) + I1 + I2
    H12 = m2*(l2**2 + L1*l2*math.cos(Pos2)) + I2
    H21 = m2*(l2**2 + L1*l2*math.cos(Pos2)) + I2
    H22 = m2*l2**2 + I2
    ## Collioris matrix elements
    V11 = -m2*L1*l2*math.sin(Pos2)*(Vel1*Vel2 + 0.5*Vel2**2)
    V21 = 0.5*m2*L1*l2*math.sin(Pos2)*Vel1**2
    ## Gravity matrix elements
    G11 = m1*g*l1*math.cos(Pos1) + m2*g*L1*math.cos(Pos1) + m2*g*l2*math.cos(Pos1 + Pos2)
    G21 = m2*g*l2*math.cos(Pos1+Pos2)

    Dynamic_error1 = 0 + theta1_error*Kp1 + error_integral1*Ki1 + vel1_error*Kv1
    Dynamic_error2 = 0 + theta2_error*Kp2 + error_integral2*Ki2 + vel2_error*Kv2

    Mass_Dynamic1 = H11*Dynamic_error1 + H12*Dynamic_error2
    Mass_Dynamic2 = H21*Dynamic_error1 + H22*Dynamic_error2

    Generate_Torque1 = Mass_Dynamic1 + V11 + G11
    Generate_Torque2 = Mass_Dynamic2 + V21 + G21

    #print("Gen Torque", Generate_Torque)
    twolink.DriveTorque(Generate_Torque1, Generate_Torque2)
    #twolink.RunPositionTorque((the1_d[-1]*180/math.pi),Generate_Torque1,(the2_d[-1]*180/math.pi),Generate_Torque2)

    # Collecting data for plot, but if there is some spike (big error) just ignore
    if abs(theta1_error) < 2 and abs(theta2_error) < 2 :

        error1.append(theta1_error*180/math.pi)
        GenTor1.append(Generate_Torque1)
        pre_the1.append(Pos1*180/math.pi)
        com_the1.append(the1_d[-1]*180/math.pi)

        error2.append(theta2_error*180/math.pi)
        GenTor2.append(Generate_Torque2)
        pre_the2.append(Pos2*180/math.pi)
        com_the2.append(the2_d[-1]*180/math.pi)
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
plt.subplot(3, 1, 1)
plt.plot(plotTime,error1,'r')
plt.grid()
plt.title("error1 [deg]")
plt.subplot(3, 1, 2)
plt.plot(plotTime,GenTor1,'b')
plt.grid()
plt.title("generated torque1 [N.m]")
plt.subplot(3,1,3)
plt.plot(plotTime,pre_the1,'g')
plt.plot(plotTime,com_the1,'k--')
plt.grid()
plt.title("Angle1 [deg]")
plt.legend(('output angle1','command angle1'), loc='lower right')

plt.figure(2)
#plt.rcParams.update({'font.size': 18})
plt.subplot(3, 1, 1)
plt.plot(plotTime,error2,'r')
plt.grid()
plt.title("error2 [deg]")
plt.subplot(3, 1, 2)
plt.plot(plotTime,GenTor2,'b')
plt.grid()
plt.title("generated torque2 [N.m]")
plt.subplot(3,1,3)
plt.plot(plotTime,pre_the2,'g')
plt.plot(plotTime,com_the2,'k--')
plt.grid()
plt.title("Angle2 [deg]")
plt.legend(('output angle2','command angle2'), loc='lower right')


plt.show()



#totalTime = time.time() - startTime
#print(totalTime)  
