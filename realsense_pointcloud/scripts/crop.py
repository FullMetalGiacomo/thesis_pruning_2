#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Float64MultiArray
import numpy as np
from dynamic_reconfigure.server import Server
from realsense_pointcloud.cfg import Reconfig_paramsConfig

class Crop(object):
    def __init__(self):
        # Params
        # self.known_w = 0.5 #In cm
        # self.cam_inf=rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        # self.focal_length=self.cam_inf.K[0]
        self.cropParam=[0.2,0.4,0.2,0.2] #top, right, bottom, left
        self.crop_box=[0.0,0.0]
        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(10)

        # Publishers
        self.rgb_pub = rospy.Publisher('/rgb_crop',  Image, queue_size=1)
        self.depth_map_pub = rospy.Publisher('/rs_depth_crop',  Image, queue_size=1)
        self.crop_par=rospy.Publisher('/crop_param',Float64MultiArray,queue_size=1)
        # Subscribers

        rospy.Subscriber("/camera/color/image_raw",Image,self.rgb_callback)
        # rospy.Subscriber("/camera/aligned_depth_to_color/image_raw/compressed",Image,self.depth_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.depth_callback)

    def recon_callback(self, config,level):
        # rospy.loginfo("""Reconfigure Request: {left_crop_param}, {right_crop_param},{top_crop_param}, {bottom_crop_param}""".format(**config))

        self.cropParam=["{top_crop_param}".format(**config),"{right_crop_param}".format(**config),"{bottom_crop_param}".format(**config),"{left_crop_param}".format(**config)]
        self.cropParam=list(np.float_(self.cropParam))

        # print(self.cropParam)
        return config
    def rgb_callback(self, data):
        image=np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # cv2.imshow('image',image)
        # cv2.waitKey(0)
        h, w, c=np.shape(image)
        xl=int(w*self.cropParam[3])
        xr=int(w-w*self.cropParam[1])
        yu=int(h*self.cropParam[0])
        yd=int(h - h*self.cropParam[2])
        self.crop_box=[yu,xl]
        # xl=int(data.width*self.cropParam[3])
        # xr=int(data.width-data.width*self.cropParam[1])
        # yu=int(data.height*self.cropParam[0])
        # yd=int(data.height - data.height*self.cropParam[2])
        image=image[yu:yd,xl:xr]
        # print(image.shape)
        self.imgmsg = Image()
        self.imgmsg.data = image.flatten().tolist()
        self.imgmsg.height=image.shape[0]
        self.imgmsg.width=image.shape[1]
        self.imgmsg.encoding="rgb8"
        self.imgmsg.step=len(self.imgmsg.data) // self.imgmsg.height

        self.rgb_pub.publish(self.imgmsg)

    def depth_callback(self, data):
        image=np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        xl=int(data.width*self.cropParam[3])
        xr=int(data.width-data.width*self.cropParam[1])
        yu=int(data.height*self.cropParam[0])
        yd=int(data.height - data.height*self.cropParam[2])
        # image = np.fromstring(data.data, np.uint8)
        # rospy.logwarn(np.shape(data.data))
        # image = cv2.imdecode(image, cv2.IMREAD_GRAYSCALE)
        # h, w=np.shape(image)
        #
        # xl=int(w*self.cropParam[3])
        # xr=int(w-w*self.cropParam[1])
        # yu=int(h*self.cropParam[0])
        # yd=int(h - h*self.cropParam[2])
        image=image[yu:yd,xl:xr,:]
        # print(image.shape)
        self.imgmsg_d = Image()
        self.imgmsg_d.data = image.flatten().tolist()
        self.imgmsg_d.height=image.shape[0]
        self.imgmsg_d.width=image.shape[1]
        self.imgmsg_d.encoding="16UC1"
        self.imgmsg_d.step=len(self.imgmsg_d.data) // self.imgmsg_d.height

        self.depth_map_pub.publish(self.imgmsg_d)

    def start(self):
        rospy.loginfo("Starting...")
        srv = Server(Reconfig_paramsConfig, self.recon_callback)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            array_msg=Float64MultiArray()
            array_msg.data=self.crop_box
            self.crop_par.publish(array_msg)
            r.sleep()

if __name__ == '__main__':
	rospy.init_node('crop', anonymous=True)
	#rospy.Subscriber("/usb_cam/image_rect_color",Image,image_callback)

	my_node = Crop()
	my_node.start()
