#!/usr/bin/env python3 
import airsim
import cv2
import numpy as np
import os
import setup_path 
import time
import csv
import pandas as pd
import random
# Use below in settings.json with blocks environment

# Overall gains and parameters:
Ka=0.9
Kp=2
Kv=1
L=5
HwL=1.0
HwR=1.0
tau=0.211486816999979  #Measured from RunData.csv
BadLR=90.0
GoodToBad=20.0
BadToGood=50
P=GoodToBad
R=BadToGood
h=BadLR
PErr=P*(h)/(R+P)
print('Gamma:' + str(1-PErr/100))
TimeStep=0.01
TotalRunTime=35
AccelScale=4.0
BrakeScale=2.0

def LVInputGen(CurrTime): # Generates the input lead vehicle signal for all platoons
    if CurrTime<15: # For the first 10 seconds, accelerate
        CaccAccel=AccelScale
    # elif CurrTime<25: # For the next 3 seconds, cruise to stabilize
        # CaccAccel=0.2
    else:
        CaccAccel=-3 # Break at -3 m/s^2
    return CaccAccel

def GilbertPLFactor(GilbertState, PVal,RVal,Hval):
    Rtemp1=random.random()
    Rtemp2=random.random()
    Rtemp3=random.random()
    # print('Rand1: ' +str(Rtemp1)+' Rand2: ' +str(Rtemp2)+' Rand3: ' +str(Rtemp3))
    
    # Good state=1, Bad state=0
    if GilbertState==1:
        PLFactor=1
    else:
        if Rtemp1<=Hval/100.0:
            PLFactor=0
        else:
            PLFactor=1
    # Now change the Gilbert State for next step
    if GilbertState==1:
        if Rtemp2<=PVal/100.0:
            OutState=0
        else:
            OutState=1
    else:
        if Rtemp3<=RVal/100.0:
            OutState=1
        else:
            OutState=0
    return PLFactor, OutState

def Controller(xi,xiM,dxiM,ddxiM,currVel,PLFac,Hw): # Generates accel input to be supplied to plant
    CaccAccel=Ka*PLFac*ddxiM-Kp*(xi-xiM+L+Hw*currVel)-Kv*(currVel-dxiM)
    return CaccAccel

def Plant(CaccAccel, PlantID): # Converts CaccAccel into throttle and brake inputs for each vehicle
    # Saturation Stuff
    if CaccAccel>AccelScale:
        CaccAccel=AccelScale
    elif CaccAccel<-BrakeScale:
        CaccAccel=-BrakeScale
    
    # Now rescale to [0,1] and set them
    if CaccAccel>=0:
        car_controls.throttle=float(CaccAccel/AccelScale)
        car_controls.brake = 0
    elif CaccAccel<0:
        car_controls.throttle=0
        car_controls.brake = -1.0*float(CaccAccel/BrakeScale)
    client.setCarControls(car_controls, PlantID)
    # if PlantID=="CarLLV": # Debugging stuff
        # print(CaccAccel)
    # print(car_controls.throttle)
    return 0


# Create all cars and setup
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True, "CarLLV")
client.enableApiControl(True, "CarL1")
client.enableApiControl(True, "CarL2")
client.enableApiControl(True, "CarL3")
client.enableApiControl(True, "CarL4")
client.enableApiControl(True, "CarL5")
client.enableApiControl(True, "CarRLV")
client.enableApiControl(True, "CarR1")
client.enableApiControl(True, "CarR2")
client.enableApiControl(True, "CarR3")
client.enableApiControl(True, "CarR4")
client.enableApiControl(True, "CarR5")
car_controls= airsim.CarControls()
car_controls.is_manual_gear = False

# Initiate all GilbertStates :
GilbStateL1=1
GilbStateL2=1
GilbStateL3=1
GilbStateL4=1
GilbStateL5=1
GilbStateR1=1
GilbStateR2=1
GilbStateR3=1
GilbStateR4=1
GilbStateR5=1



startTime=time.time()
RunTime=time.time()-startTime
while RunTime<TotalRunTime: # Max Run time;
    RunTime=time.time()-startTime
    # Get all states
    StateLLV = client.getCarState("CarLLV")
    StateL1 = client.getCarState("CarL1")
    StateL2 = client.getCarState("CarL2")
    StateL3 = client.getCarState("CarL3")
    StateL4 = client.getCarState("CarL4")
    StateL5 = client.getCarState("CarL5")
    StateRLV = client.getCarState("CarRLV")
    StateR1 = client.getCarState("CarR1")
    StateR2 = client.getCarState("CarR2")
    StateR3 = client.getCarState("CarR3")
    StateR4 = client.getCarState("CarR4")
    StateR5 = client.getCarState("CarR5")
    # print(StateLLV.rpm)
    #### Calculate All the Controller outputs
    # First, for LVs:
    caccOut=LVInputGen(RunTime)
    Plant(caccOut, "CarLLV")
    Plant(caccOut, "CarRLV")
    # print(caccOut)
    
    # Now for the other FVs:

    PLFac,GilbStateL1=GilbertPLFactor(GilbStateL1,P,R,h)
    xiM=StateLLV.kinematics_estimated.position.x_val
    dxiM=StateLLV.speed
    ddxiM=StateLLV.kinematics_estimated.linear_acceleration.x_val
    caccOut=Controller(StateL1.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateL1.speed,PLFac,HwL)
    Plant(caccOut, "CarL1")

    xiM=StateL1.kinematics_estimated.position.x_val
    dxiM=StateL1.speed
    ddxiM=StateL1.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateL2=GilbertPLFactor(GilbStateL2,P,R,h)
    caccOut=Controller(StateL2.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateL2.speed,PLFac,HwL)
    Plant(caccOut, "CarL2")

    xiM=StateL2.kinematics_estimated.position.x_val
    dxiM=StateL2.speed
    ddxiM=StateL2.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateL3=GilbertPLFactor(GilbStateL3,P,R,h)
    caccOut=Controller(StateL3.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateL3.speed,PLFac,HwL)
    Plant(caccOut, "CarL3")

    xiM=StateL3.kinematics_estimated.position.x_val
    dxiM=StateL3.speed
    ddxiM=StateL3.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateL4=GilbertPLFactor(GilbStateL4,P,R,h)
    caccOut=Controller(StateL4.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateL4.speed,PLFac,HwL)
    Plant(caccOut, "CarL4")

    xiM=StateL4.kinematics_estimated.position.x_val
    dxiM=StateL4.speed
    ddxiM=StateL4.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateL5=GilbertPLFactor(GilbStateL5,P,R,h)
    # print('PLFac: '+ str(PLFac) +'GilbertState: ' +str(GilbStateL5))

    caccOut=Controller(StateL5.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateL5.speed,PLFac,HwL)
    Plant(caccOut, "CarL5")

    # For all Right side cars, force PLFac=1
    xiM=StateRLV.kinematics_estimated.position.x_val
    dxiM=StateRLV.speed
    ddxiM=StateRLV.kinematics_estimated.linear_acceleration.x_val
    caccOut=Controller(StateR1.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateR1.speed,1,HwR)
    Plant(caccOut, "CarR1")

    xiM=StateR1.kinematics_estimated.position.x_val
    dxiM=StateR1.speed
    ddxiM=StateR1.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateR2=GilbertPLFactor(GilbStateR2,P,R,h)
    caccOut=Controller(StateR2.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateR2.speed,1,HwR)
    Plant(caccOut, "CarR2")

    xiM=StateR2.kinematics_estimated.position.x_val
    dxiM=StateR2.speed
    ddxiM=StateR2.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateR3=GilbertPLFactor(GilbStateR3,P,R,h)
    caccOut=Controller(StateR3.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateR3.speed,1,HwR)
    Plant(caccOut, "CarR3")

    xiM=StateR3.kinematics_estimated.position.x_val
    dxiM=StateR3.speed
    ddxiM=StateR3.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateR4=GilbertPLFactor(GilbStateR4,P,R,h)
    caccOut=Controller(StateR4.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateR4.speed,1,HwR)
    Plant(caccOut, "CarR4")

    xiM=StateR4.kinematics_estimated.position.x_val
    dxiM=StateR4.speed
    ddxiM=StateR4.kinematics_estimated.linear_acceleration.x_val
    PLFac,GilbStateR5=GilbertPLFactor(GilbStateR5,P,R,h)
    caccOut=Controller(StateR5.kinematics_estimated.position.x_val,xiM,dxiM,ddxiM,StateR5.speed,1,HwR)
    Plant(caccOut, "CarR5")

    # Now just sleep so the cars are allowed to move
    time.sleep(TimeStep)


#restore to original state
print(StateRLV.kinematics_estimated.position.x_val)
client.reset()

client.enableApiControl(False)

print('done')

            
