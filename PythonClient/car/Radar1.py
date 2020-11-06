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
def CamMain():
    # for car use CarClient() 
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True, "Car")
    ObjSrcList=client.simListSceneObjects(name_regex='SM.*')
    print(ObjSrcList)
    EgoPose=client.simGetObjectPose('Car')
    # print(EgoPose.)
    ObjFilteredList=[]
    ObjListPos=[]
    ObjListOrn=[]
    startTime=time.time()
    # for idx in range(len(ObjSrcList)): # Get position and offset it, store orientation as well
    #     if ("SM" in ObjSrcList[idx]):
    #         ObjFilteredList.append(ObjSrcList[idx])

    # print(ObjFilteredList)
    for jdx in range(len(ObjSrcList)): 
        tempPose=client.simGetObjectPose(ObjSrcList[jdx])
        # ObjListPos.append(tempPose.position-EgoPose.position)
        print(jdx)
    print(time.time()-startTime)
    # ThermalPub = rospy.Publisher("Thermal_Airsim",Image,queue_size=100)
    rate = rospy.Rate(20) # 10hz
    print('Done')
    # while not rospy.is_shutdown():
        # pass
        # png_image = client.simGetImage("0", airsim.ImageType.Scene)
        # responses = client.simGetImages([airsim.ImageRequest("IR_Center", airsim.ImageType.Scene, False, False)])
        # response=responses[0]
        # img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        # img_rgb = img1d.reshape(response.height,response.width, 3)

        # #Convert to grayscale to simulate thermal, convert image back to 3 channel format and publish:
        # img_gray= cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        # img_rgb2=cv2.cvtColor(img_gray,cv2.COLOR_GRAY2RGB) 
        # ThermalPub.publish(bridge.cv2_to_imgmsg(img_rgb2, "mono8"))
        # rate.sleep()
   

if __name__=='__main__':
    rospy.init_node('airsim_thermal', anonymous=True)
    try:
        CamMain()
    except rospy.ROSInterruptException:
        pass
#######################
