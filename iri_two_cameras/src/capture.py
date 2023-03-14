#!/usr/bin/env python3

import cv2
import os
import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Float64MultiArray
import numpy as np
from dynamic_reconfigure.server import Server
from iri_two_cameras.cfg import Reconfig_paramsConfig
import message_filters

class Capture(object):

    # def __init__(self):
    #

    def recon_callback(self, config,level):
        self.capture=("{capture}".format(**config))
        config.__setitem__("capture",False)
        return config

    def image_callback(self, one,two):
        rp = rospkg.RosPack()
        package_path = rp.get_path('iri_two_cameras')
        if self.capture == "True":
            rospy.logwarn(os.path.join(package_path,"Camera1" , str(self.i)+'.jpg'))
            rgb_im_1=np.frombuffer(one.data, dtype=np.uint8).reshape(one.height, one.width, -1)
            rgb_im_1 = cv2.cvtColor(rgb_im_1, cv2.COLOR_BGR2RGB)
            cv2.imwrite(os.path.join(package_path,"Camera1" , str(self.i)+'.jpg'), rgb_im_1)
            rgb_im_2=np.frombuffer(two.data, dtype=np.uint8).reshape(two.height, two.width, -1)
            rgb_im_2 = cv2.cvtColor(rgb_im_2, cv2.COLOR_BGR2RGB)
            cv2.imwrite(os.path.join(package_path, "Camera2" , str(self.i)+'.jpg'), rgb_im_2)
            self.capture="False"
            self.i=self.i+1


    def start(self):
        rospy.loginfo("Starting...")
        srv = Server(Reconfig_paramsConfig, self.recon_callback)
        one = message_filters.Subscriber('/camera_0/color/image_raw', Image)
        two = message_filters.Subscriber('/camera_1/color/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([one, two],30,0.01)
        self.i=0
        ts.registerCallback(self.image_callback)
        r = rospy.Rate(15)
        rospy.spin()

if __name__ == '__main__':
	rospy.init_node('capture_two_cameras', anonymous=True)
	my_node = Capture()
	my_node.start()
