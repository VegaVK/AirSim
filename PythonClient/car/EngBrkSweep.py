import airsim
import cv2
import numpy as np
import os
import setup_path 
import time
import csv
import pandas as pd
# Use below in settings.json with blocks environment


# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True, "Car1")

car_controls = airsim.CarControls()
car_controls.is_manual_gear = False
ThrList=[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
BrkList=[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
startTime=time.time()
TimeStep=0.01
#Create data frame with cols: Time,Thr,Brake,Vel,Accel,GearNumber,
FirstRow=[[0,0,0,0,0,0,0]]
df=pd.DataFrame(FirstRow,columns =['Time','Throttle','Brake','Vel','Accel','Gear','RPM'])
# df.append({'Time' : 1, 'Throttle' : 97, 'Brake' : 2200,'Vel' : 2200,'Accel' : 2200,'Gear' : 2200},ignore_index = True) 
for idx in range(10):
    
    ThrVal=ThrList[idx]
    BrkVal=BrkList[idx]
    # get state of the car
    CurrState = client.getCarState("Car1")
    runStartTime=time.time()
    RunTime=time.time()-runStartTime

    while (CurrState.speed<35) and (RunTime<30): # Each run Shouldn't take longer than 30s
        # accelerate
        car_controls.throttle = ThrVal
        car_controls.brake = 0
        client.setCarControls(car_controls, "Car1")
        time.sleep(TimeStep)
        CurrTime=time.time()-startTime
        RunTime=time.time()-runStartTime
        CurrState = client.getCarState("Car1")
        # print(CurrState.speed)
        # print(CurrTime)
        df=df.append({'Time' : CurrTime, 'Throttle' : ThrVal, 'Brake' : 0,'Vel' : CurrState.speed,\
            'Accel' : CurrState.kinematics_estimated.linear_acceleration.x_val ,'Gear' : CurrState.gear,'RPM':CurrState.rpm },ignore_index = True) 
    # Now brake to come to stop
    while CurrState.speed>0.01:
        car_controls.brake = BrkVal
        car_controls.throttle = 0
        client.setCarControls(car_controls, "Car1")
        time.sleep(TimeStep)
        CurrTime=time.time()-startTime
        CurrState = client.getCarState("Car1")
        df=df.append({'Time' : CurrTime, 'Throttle' : 0, 'Brake' : BrkVal,'Vel' : CurrState.speed,\
            'Accel' : CurrState.kinematics_estimated.linear_acceleration.x_val ,'Gear' : CurrState.gear,'RPM':CurrState.rpm },ignore_index = True)
    
    print('idx:'+str(idx))
   
#restore to original state
client.reset()

client.enableApiControl(False)
# Write to csv:
df.to_csv('RunData.csv')
print('done')


            
