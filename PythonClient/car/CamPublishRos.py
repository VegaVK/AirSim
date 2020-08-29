import airsim #pip install airsim
import rospy
import std_msgs
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np
def CamMain():
    # for car use CarClient() 
    client = airsim.CarClient()
    bridge=CvBridge()
    ThermalPub = rospy.Publisher("Thermal_Airsim",Image,queue_size=100)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        # png_image = client.simGetImage("0", airsim.ImageType.Scene)
        responses = client.simGetImages([airsim.ImageRequest("IR_Center", airsim.ImageType.Infrared, False, False)])
        response=responses[0]
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        img_rgb = img1d.reshape(response.height,response.width, 3)
        # original image is fliped vertically
        # img_rgb = np.fliplr(img_rgb)
        # print(img_rgb)
        ThermalPub.publish(bridge.cv2_to_imgmsg(img_rgb, "rgb8"))
        rate.sleep()
   

if __name__=='__main__':
    rospy.init_node('airsim_thermal', anonymous=True)
    try:
        CamMain()
    except rospy.ROSInterruptException:
        pass
#######################
