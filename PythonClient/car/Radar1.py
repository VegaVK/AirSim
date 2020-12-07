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
from scipy.spatial.transform import Rotation as R
# import visualization_msgs 
from visualization_msgs.msg import Marker

def CamMain():
    # for car use CarClient()
    #=====PARAMETERS:
    PD=1 # Prob of detection 
    NoiseSD_x=0
    NoiseSD_y=0
    client = airsim.CarClient()
    client.confirmConnection()
    # client.enableApiControl(True, "Car")
    ObjStatList=client.simListSceneObjects(name_regex='SF.*') # Just cars, for testing in Neighborhood Scene
    # ObjStatList=client.simListSceneObjects(name_regex='Car.*|Tree.*') # Just Car, for testing in Neighborhood Scene
    # ObjStatList=client.simListSceneObjects(name_regex='Car.*') # Just cars, for testing in Neighborhood Scene
    # ObjStatList=client.simListSceneObjects() # All Objects in scene
    # print(ObjStatList)

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
        EgoTrnfMat= R.from_quat([EgoPose.orientation.x_val,EgoPose.orientation.y_val,EgoPose.orientation.z_val,EgoPose.orientation.w_val])
        EgoTrnfMat=np.vstack((np.hstack((np.array(EgoTrnfMat.as_dcm()),np.array([EgoPose.position.x_val,EgoPose.position.y_val,EgoPose.position.z_val]).reshape((3,1)))),np.array([0,0,0,1])))
        # print(EgoTrnfMat)
        ReturnPositionList=AddPosNoise(UpdateRelativePose(client,ObjStatList,EgoTrnfMat,EgoPose),PD,NoiseSD_x,NoiseSD_y)
        
        RadarMarkerPublisher(ReturnPositionList,RdrMarkerPub)
        rate.sleep()
   
def UpdateRelativePose(client,InputList,EgoTrnfMat,EgoPose):
    ReturnList=[] # For now, just position, #TODO: relative vel and orientation
    # ObjStatList=client.simListSceneObjects(name_regex='Car.*|Tree.*') # Cars and Trees, for testing in Neighborhood Scene
    ObjStatList=client.simListSceneObjects(name_regex='SF.*') # Just cars, for testing in Neighborhood Scene
    # EgoPose=client.simGetObjectPose('Car')
    print(EgoPose.orientation)
    for itemDx in range(len(InputList)): 
        objPose=client.simGetObjectPose(InputList[itemDx])
        
        # print(itemDx)
        # print(ObjStatList[itemDx])
        # print(objPose.orientation)
        objTrnfMat= R.from_quat([EgoPose.orientation.x_val,EgoPose.orientation.y_val,-EgoPose.orientation.z_val,EgoPose.orientation.w_val]) 
        # Invert it, to get direction cosines of I_hat, J_hat... of F_world in F_Car frame:
        objTrnfMat=np.linalg.inv(objTrnfMat.as_dcm())
        netTrnfMat=np.vstack((np.hstack((np.array(objTrnfMat),np.array([EgoPose.position.x_val,EgoPose.position.y_val,0]).reshape((3,1)))),np.array([0,0,0,1])))
        objRelativePosVector=np.matmul(netTrnfMat,np.array([-objPose.position.y_val,-objPose.position.x_val,objPose.position.z_val,1]).reshape((4,1)))
        # print(temp)
        ReturnList.append(objRelativePosVector[:-1])
        # print(ReturnList[-1][0])
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
            tempPose[0]=tempPose[0]+noise_x[itemDx]
            tempPose[1]=tempPose[1]+noise_y[itemDx]
            ReturnList.append(tempPose)
        else:
            pass
    return ReturnList

## TODO: NEED THE FOLLOWING :
# self.RdrReadings[-1].pose=data.objects[idx].pose.pose
#             self.RdrReadings[-1].vx=data.objects[idx].twist.twist.linear.x
#             self.RdrReadings[-1].vy=data.objects[idx].twist.twist.linear.y
#             self.RdrReadings[-1].vx_comp=self.velX+data.objects[idx].twist.twist.linear.x
#             self.RdrReadings[-1].vy_comp=self.velY+data.objects[idx].twist.twist.linear.y
#             self.RdrReadings[-1].header=data.objects[idx].header
#             self.RdrReadings[-1].id=data.objects[idx].id

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
        
        tempPoint.x=itemDxj[0]
        tempPoint.y=itemDxj[1]
        tempPoint.z=itemDxj[2]
        if any(np.isnan([itemDxj[0],itemDxj[1],itemDxj[2]])):
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