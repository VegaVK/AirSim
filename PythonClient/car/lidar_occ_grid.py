#!/usr/bin/env python3 
import rospy
import setup_path 
import airsim
import cv2
import numpy as np
import os
import sys
import math
import setup_path 
import argparse
import pprint
import time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Use below in settings.json with blocks environment

# Overall gains and parameters:

TimeStep=0.01
TotalRunTime=60
#  Simple Velocity Tracker, Proportional only
Kp=0.3
TargetVel=5 # m/s
AccelScale=4.0
BrakeScale=2.0
binSize=0.2 # in meters, binning size for occupancy grid, for use on lidar data
rospy.init_node('ramp_merge', anonymous=True)
RdrMarkerPub = rospy.Publisher("radar_markers_rviz",Marker,queue_size=100)
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

def parse_lidarData(data):

        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

def filter_and_bin(points):
    # Filter out all points on the ground and then bin them to the required resolution. 
    toDelList=[]
    for pointIdx in range(len(points)):
        # print(points[pointIdx])
        # print(points[pointIdx][2])
        if (points[pointIdx][2]<=-2) or (points[pointIdx][2]>=0.8) or (points[pointIdx][0]>=30) \
            or (points[pointIdx][1]>=30): # Z-axis from pointcloud is inverted
            # print(points[pointIdx][2])
            # print('APPENDING idx' + str(pointIdx))
            toDelList.append(pointIdx)
    # print(toDelList)
    # print('Before Filtering: '+ str(len(points)))
    points=np.delete(points,toDelList,axis=0)
    # print('After: ' +str(len(points)))

    scaleFactor=1/binSize
    # First scale all points, floor them and then rescale
    points=points*scaleFactor
    points=np.floor(points)
    points=points/scaleFactor
    return points
    

def RadarMarkerPublisher(InputList,RadarPublisher):
    # MarkerArrayIn=visualization_msgs.msg.MarkerArray()
    markerTemp=Marker()
    markerTemp.header.frame_id = "map"
    markerTemp.type = markerTemp.CUBE_LIST
    markerTemp.action = markerTemp.ADD
    markerTemp.scale.x = 0.4
    markerTemp.scale.y = 0.4
    markerTemp.scale.z = 0.2
    markerTemp.color.r = 0.0
    markerTemp.color.g = 1.0
    markerTemp.color.b = 1.0
    markerTemp.color.a = 1.0
    for itemDxj in InputList:
        tempPoint=Point()
        
        tempPoint.x=itemDxj[0]
        tempPoint.y=-itemDxj[1]
        tempPoint.z=0
        if any(np.isnan([itemDxj[0],itemDxj[1],itemDxj[2]])):
            # print('found NaN')
            pass
        else:
            markerTemp.points.append(tempPoint)
    RadarPublisher.publish(markerTemp)
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
    # print(StateRLV)
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

    # Get Lidar Data:
    lidarData = client.getLidarData( lidar_name = 'LidarSensor1', vehicle_name = 'CarFPV')
    if (len(lidarData.point_cloud) < 3):
        print("\tNo points received from Lidar data")
    else:
        points = parse_lidarData(lidarData)
        points=filter_and_bin(points)
        RadarMarkerPublisher(points,RdrMarkerPub)
        # print("\tTime_stamp: %d number_of_points: %d" % (lidarData.time_stamp, len(points)))
        # print(points[0])
        # print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
        # print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
    


#restore to original state
client.reset()

client.enableApiControl(False)

print('done')

            
