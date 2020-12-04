#!/usr/bin/env python
# Code snippet to get pose of all objects in scene
import sys
import airsim #pip install airsim
import rospy
import std_msgs
import numpy as np
import time
from geometry_msgs.msg import Point

def getPose():

    client = airsim.CarClient()
    client.confirmConnection()
    # Get all objects
    startTime=time.time()
    ObjList=client.simListSceneObjects()
    # Loop over list for postions
    poseList=[]
    for item in range(len(ObjList)): 
        tempPose=client.simGetObjectPose(ObjList[item])
        ObjList.append(tempPose.position)
    print('Took ' + str(time.time()-startTime) +'s to complete')
    # print(ObjList)
if __name__=='__main__':
    getPose()

