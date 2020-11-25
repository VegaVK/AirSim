#!/usr/bin/env python
import sys
# add the path to the folder that contains the AirSimClient module
sys.path += [ "/home/vamsi/Unreal/Airsim/PythonClient" ]
# print(sys.path)
# now this import should succeed
# from AirSimClient import *
import airsim #pip install airsim
import rospy
import std_msgs
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np
import time
from geometry_msgs.msg import Point
import random
# import visualization_msgs 
from visualization_msgs.msg import Marker

def CamMain():
    # for car use CarClient()
    PD=0.9 # Prob of detection 
    client = airsim.CarClient()
    client.confirmConnection()
    # client.enableApiControl(True, "Car")
    # ObjStatList=client.simListSceneObjects(name_regex='SF.*') # Static list for fusion
    ObjStatList=client.simListSceneObjects(name_regex='Car.*|Tree.*') # Just Car, for testing
    # ObjStatList=client.simListSceneObjects() # All Objects in scene
    print(ObjStatList)
    
    ObjListPos=[]
    ObjListOrn=[]
    startTime=time.time()
    # for idx in range(len(ObjSrcList)): # Get position and offset it, store orientation as well
    #     if ("SM" in ObjSrcList[idx]):
    #         ObjFilteredList.append(ObjSrcList[idx])
    for jdx in range(len(ObjStatList)): 
        tempPose=client.simGetObjectPose(ObjStatList[jdx])
        # ObjListPos.append(tempPose.position-EgoPose.position)
        # print(jdx)
    # print(time.time()-startTime)
    print('Created Static Object List')
    #TODO: Create a ObjDynList that cycles though moving items

    #Now translate all to relative coords and create radar markers for ROS

    RdrMarkerPub = rospy.Publisher("radar_markers_rviz",Marker,queue_size=100)
    rate = rospy.Rate(20) # 10hz
    # print('Done')
    while not rospy.is_shutdown():
        EgoPose=client.simGetObjectPose('Car')
        # print(EgoPose)
        ReturnPosList=AddPosNoise(UpdateRelativePose(client,ObjStatList,EgoPose),PD,0.25,0.25)
        RadarMarkerPublisher(ReturnPosList,RdrMarkerPub)
        rate.sleep()
   
def UpdateRelativePose(client,InputList,EgoPose):
    ReturnList=[] # For now, just position, #TODO: relative vel and orientation
    for itemDx in range(len(InputList)): 
        tempPose=client.simGetObjectPose(InputList[itemDx])
        ReturnList.append(tempPose.position-EgoPose.position)
    return ReturnList
#TODO: Baselink/frame stuff

def AddPosNoise(InputList,PD,NoiseSD_x,NoiseSD_y):
    ReturnList=[] # For now, just position, #TODO: relative vel and orientation
    noise_x = np.random.normal(0,NoiseSD_x,len(InputList))
    noise_y = np.random.normal(0,NoiseSD_y,len(InputList))
    for itemDx in range(len(InputList)): 
        RandVal=random.random()
        if RandVal<= PD: # Object is detected:
            tempPose=InputList[itemDx]
            tempPose.x_val=tempPose.x_val+noise_x[itemDx]
            tempPose.y_val=tempPose.y_val+noise_y[itemDx]
            ReturnList.append(tempPose)
        else:
            pass
    return ReturnList



def RadarMarkerPublisher(InputList,RadarPublisher):
    # MarkerArrayIn=visualization_msgs.msg.MarkerArray()
    markerTemp=Marker()
    markerTemp.header.frame_id = "map"
    markerTemp.type = markerTemp.CUBE_LIST
    markerTemp.action = markerTemp.ADD
    markerTemp.scale.x = 0.2
    markerTemp.scale.y = 0.2
    markerTemp.scale.z = 0.2
    markerTemp.color.r = 0.0
    markerTemp.color.g = 1.0
    markerTemp.color.b = 1.0
    markerTemp.color.a = 1.0
    for itemDxj in InputList:
        tempPoint=Point()
        
        tempPoint.x=itemDxj.x_val
        tempPoint.y=itemDxj.y_val
        tempPoint.z=itemDxj.z_val
        if any(np.isnan([itemDxj.x_val,itemDxj.y_val,itemDxj.z_val])):
            # print('found NaN')
            pass
        else:
            markerTemp.points.append(tempPoint)
    RadarPublisher.publish(markerTemp)



if __name__=='__main__':
    rospy.init_node('airsim_thermal', anonymous=True)
    try:
        CamMain()
    except rospy.ROSInterruptException:
        pass
#######################
