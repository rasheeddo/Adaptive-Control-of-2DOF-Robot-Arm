import time
from TwoLinkRobotClass import *
import math

twolink  = TwoLinkRobot()

#twolink.SetOperatingMode(1,twolink.CURRENT_CONTROL)
#twolink.SetOperatingMode(2,twolink.CURRENT_CONTROL)

########################################################## Note ####################################################################
## Using CURRENT_CONTROL will give smooth motion when user push/pull the robot, but when implement with other mode like tracking trajectory (in the future), 
## CURRENT BASED POSITION CONTROLL would be better, so in this mode, we are trying to use the same CONTROL mode.
## SetPID ----- the value of P should not be lower than 1000 because the robot cannot hold the position when steady
##              the value of D would give the resistive force to user push/pull, so if you want the robot to be soft, easily to push, just set D to 0

## In While loop, there are if condition to check the present and last position, if the difference is larger than 0.4, that means user try to push,
## so just drive to robot to that present position with that torque
## don't need delay time at the end, it will make the system lack of reading and compensate the motion
#####################################################################################################################################

twolink.SetOperatingMode(1,twolink.CURRENT_BASED_POSITION_CONTROL)
twolink.SetOperatingMode(2,twolink.CURRENT_BASED_POSITION_CONTROL)

twolink.SetPID(1,1000,0,0) # PID gain for servo position loop control (1,900,0,1000)
twolink.SetPID(2,1000,0,0)

twolink.TorqueOn()

#twolink.DriveTorque(twolink.InitTor1,twolink.InitTor2)
#twolink.RunPositionTorque(0,twolink.InitTor1,0,twolink.InitTor2)

M1 = twolink.M1
g = twolink.g
L1 = twolink.L1
M2 = twolink.M2
L2 = twolink.L2
l1 = twolink.l1
l2 = twolink.l2

PI = 3.141592654
deg2rad = 0.01745329252   # x pi/180

print("Start at robot 0 deg...")

time.sleep(1)

T = time.time()
Ts = 0.02 # sampling time 2ms
LastAng1 = 0
LastAng2 = 0
PreAng1 = 0
PreAng2 = 0

while True:
    T=T+Ts

    # Check present angle
    PreAng1, PreAng2 = twolink.ReadAngle()
    #Tau1, Vel1, PreAng1, Tau2, Vel2, PreAng2 = twolink.ReadTorVelPos()

    print("PreAng1",PreAng1)
    print("PreAng2",PreAng2)
    ReqTor1 = M1*g*l1*math.cos(PreAng1*deg2rad) + M2*g*L1*math.cos(PreAng1*deg2rad) + M2*g*l2*math.cos((PreAng1+PreAng2)*deg2rad)
    ReqTor2 = M2*g*l2*math.cos((PreAng1+PreAng2)*deg2rad)

    print("ReqTor1",ReqTor1)
    #print("Tau1", Tau1)
    print("ReqTor2",ReqTor2)
    #rint("Tau2", Tau1)
    print("-------------------------------------")
    #twolink.DriveTorque(ReqTor1,ReqTor2)


    # Check some difference between last and present position
    # to not write the flow of command to servo
    if abs(PreAng1 - LastAng1) > 0.4:
        #RunPositionTorque(PreAng,ReqTor)
        twolink.RunPositionTorque(PreAng1,ReqTor1,LastAng2,ReqTor2)
        print("---------------- Drive 1 ----------------")
    if abs(PreAng2 - LastAng2) > 0.4:
        twolink.RunPositionTorque(LastAng1,ReqTor1,PreAng2,ReqTor2)
        print("---------------- Drive 2 ----------------")

    
    LastAng1 = PreAng1
    LastAng2 = PreAng2
    #print("period %.6f" %period)
    
    delayTime = max(0,T-time.time())
    #time.sleep(delayTime)     #max is needed in Windows due to
                            #sleep's behaviour with negative argument