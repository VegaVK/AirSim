#!/usr/bin/env python3 
import airsim
import cv2
import numpy as np
import os
import setup_path 
import time

# Use below in settings.json with blocks environment

# Overall gains and parameters:

TimeStep=0.01
TotalRunTime=60
#  Simple Velocity Tracker, Proportional only
Kp=0.3
TargetVel=5 # m/s
AccelScale=4.0
BrakeScale=2.0
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
# client.enableApiControl(True, "CarRLV")
client.enableApiControl(True, "CarR1")
client.enableApiControl(True, "CarR2")
client.enableApiControl(True, "CarR3")
client.enableApiControl(True, "CarR4")
client.enableApiControl(True, "CarR5")
client.enableApiControl(True, "CarR6")
client.enableApiControl(True, "CarR7")
client.enableApiControl(True, "CarR8")
client.enableApiControl(True, "CarR9")
client.enableApiControl(True, "CarR10")
car_controls= airsim.CarControls()
car_controls.is_manual_gear = False




startTime=time.time()
RunTime=time.time()-startTime

while RunTime<TotalRunTime: # Max Run time;
    RunTime=time.time()-startTime
    # Get all states
    StateRLV = client.getCarState("CarFPV")
    print(StateRLV)
    StateR1 = client.getCarState("CarR1")
    StateR2 = client.getCarState("CarR2")
    StateR3 = client.getCarState("CarR3")
    StateR4 = client.getCarState("CarR4")
    StateR5 = client.getCarState("CarR5")
    StateR6 = client.getCarState("CarR6")
    StateR7 = client.getCarState("CarR7")
    StateR8 = client.getCarState("CarR8")
    StateR9 = client.getCarState("CarR9")
    StateR10 = client.getCarState("CarR10")

    accelReq= Kp*(TargetVel-StateR1.speed)
    Plant(accelReq, "CarR1")
    accelReq= Kp*(TargetVel-StateR2.speed)
    Plant(accelReq, "CarR2")
    accelReq= Kp*(TargetVel-StateR3.speed)
    Plant(accelReq, "CarR3")
    accelReq= Kp*(TargetVel-StateR4.speed)
    Plant(accelReq, "CarR4")
    accelReq= Kp*(TargetVel-StateR5.speed)
    Plant(accelReq, "CarR5")
    accelReq= Kp*(TargetVel-StateR6.speed)
    Plant(accelReq, "CarR6")
    accelReq= Kp*(TargetVel-StateR7.speed)
    Plant(accelReq, "CarR7")
    accelReq= Kp*(TargetVel-StateR8.speed)
    Plant(accelReq, "CarR8")
    accelReq= Kp*(TargetVel-StateR9.speed)
    Plant(accelReq, "CarR9")
    accelReq= Kp*(TargetVel-StateR10.speed)
    Plant(accelReq, "CarR10")
    
    # Now just sleep so the cars are allowed to move
    time.sleep(TimeStep)


#restore to original state
client.reset()

client.enableApiControl(False)

print('done')

            
