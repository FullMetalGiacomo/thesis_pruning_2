#!/usr/bin/env python3

import sys
import math

import cv2
from matplotlib import pyplot as plt
import rospy
# import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Float64MultiArray
import numpy as np
from dynamic_reconfigure.server import Server

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# roslaunch realsense2_camera rs_camera.launch align_depth:=true mode:=manualcolor_fps:=15 color_width:=1280 color_height:=720 depth_fps:=15 depth_width:=1280 depth_height:=720 enable_pointcloud:=True flters:=spatial,temporal,hole_filling,decimation,disparity

class pcl_preprocesser(object):
    def __init__(self):
        # Params
        # self.known_w = 0.5 #In cm
        # self.cam_inf=rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        # self.focal_length=self.cam_inf.K[0]
        # self.cropParam=[0.2,0.4,0.2,0.2] #top, right, bottom, left
        # self.crop_box=[0.0,0.0]
        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(10)
        self.bridge = CvBridge()
        # Publishers
        # self.rgb_pub = rospy.Publisher('/rgb_crop',  Image, queue_size=1)
        self.depth_map_pub = rospy.Publisher('/filtered_depth_image',  Image, queue_size=1)
        # self.depth_cables_pub = rospy.Publisher('/filtered_depth_cables',  Image, queue_size=1)
        # self.crop_par=rospy.Publisher('/crop_param',Float64MultiArray,queue_size=1)
        # self.stelldaten = rospy.Publisher('processed_pcl', PointCloud2, queue_size=10)

        # Subscribers
        # rospy.Subscriber("/passthrough/output", PointCloud2,self.pcl_callback,queue_size=1, buff_size=52428800)


    # def recon_callback(self, config,level):
    #     # rospy.loginfo("""Reconfigure Request: {left_crop_param}, {right_crop_param},{top_crop_param}, {bottom_crop_param}""".format(**config))
    #
    #     self.cropParam=["{top_crop_param}".format(**config),"{right_crop_param}".format(**config),"{bottom_crop_param}".format(**config),"{left_crop_param}".format(**config)]
    #     self.cropParam=list(np.float_(self.cropParam))
    #
    #     # print(self.cropParam)
    #     return config

    def reading_callback(self,color_image_rect, depth_image_rect):
    # Solve all of perception here...
        ################################################# READING
        color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
        depth_image_rect_copy=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)
        depth_image_rect_copy_cables=np.copy(depth_image_rect_copy)

        # cv2.imshow('color_image_rect',color_image_rect)
        # cv2.waitKey(0)
        ################################################# PLANT SEGMENTATION

        # light removal
        lightness_thresh=100; # dynamic param should be implemented
        # HSL_MinLight = np.array([0,  0, 0],np.uint8)
        # HSL_MaxLight = np.array([255, lightness_thresh, 255],np.uint8)
        HLS_image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2HLS_FULL)
        # cv2.imshow('HLS_image',HLS_image)
        # cv2.waitKey(0)
        # HLS_image_without_lightness = cv2.inRange(HLS_image, HSL_MinLight, HSL_MaxLight)
        # print(HLS_image_without_lightness)
        # print(type(HLS_image_without_lightness))
        # print(HLS_image_without_lightness.size)
        # cv2.imshow('HLS_image_whithout_lightness',HLS_image_without_lightness)
        # cv2.waitKey(0)
        # ret,HLS_image_without_lightness_thresh = cv2.threshold(HLS_image_without_lightness,127,255,cv2.THRESH_BINARY)
        # cv2.imshow('HLS_image_without_lightness_thresh',HLS_image_without_lightness_thresh)
        # cv2.waitKey(0)
        # HLS_image_without_lightness_bool=np.array([HLS_image_without_lightness_thresh==1])
        # HLS_image_without_lightness_bool=np.squeeze(HLS_image_without_lightness_bool, axis=0)
        # print(HLS_image_without_lightness_bool)
        # print(type(HLS_image_without_lightness_bool))
        # print(HLS_image_without_lightness_bool.shape)


        # blue removal
        # HSV_image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2HSV)
        # cv2.imshow('HSV_image',HSV_image)
        # cv2.waitKey(0)
        # redChannel = np.copy(color_image_rect[:, :, 0])
        # greenChannel = np.copy(color_image_rect[:, :, 1])
        # blueChannel = np.copy(color_image_rect[:, :, 2])
        # holder1 = blueChannel>greenChannel
        # holder2 = blueChannel>redChannel
        # predominant_blue_pixels_bool= holder1 & holder2
        # print(predominant_blue_pixels_bool)
        # print(type(predominant_blue_pixels_bool))
        # print(predominant_blue_pixels_bool.shape)
        # print(predominant_blue_pixels)
        # print(type(predominant_blue_pixels))
        # predominant_blue_pixels=np.uint8(predominant_blue_pixels_bool)*255

        # test=np.array([np.array([blueChannel>greenChannel]) & np.array([blueChannel>redChannel]]))
        # test = test.astype(np.uint8)
        # HSV_MinBlue = np.array([90,  200, 200],np.uint8)
        # HSV_MaxBlue = np.array([100, 255, 255],np.uint8)
        # HSV_image_without_blue = cv2.inRange(HSV_image, HSV_MinBlue, HSV_MaxBlue)
        # cv2.imshow('Blue channel', blueChannel)
        # cv2.imshow('Green channel', greenChannel)
        # cv2.imshow('Red channel', redChannel)
        # cv2.imshow('predominant_blue_pixels', predominant_blue_pixels)
        # cv2.imshow('HSV_image_without_blue',HSV_image_without_blue)
        # cv2.waitKey(0)
        # print("hello HLS")
        # print(np.shape(HLS_image))
        # print(depth_image_rect)
        # print(np.shape(depth_image_rect))
        # print(np.shape(HLS_image[:,:,1]>150))
        # print(HLS_image[:,:,1]>150)


        #depth_image_rect_copy[np.invert(HLS_image_without_lightness_bool)]=0 # removing lightness points

        depth_image_rect_copy=np.copy(depth_image_rect_copy)

        brown_color_filter=((color_image_rect[:,:,0]<242) & (color_image_rect[:,:,1]<222) & (color_image_rect[:,:,2]<202)) # selecting browns
        # reduces effect of infrared
        #blue_color_filter=((color_image_rect[:,:,0]<60) & (color_image_rect[:,:,1]<150) & (color_image_rect[:,:,2]<255)) # selecting blues

        depth_image_rect_copy[HLS_image[:,:,1]>lightness_thresh]=0 # removing lightness points
        depth_image_rect_copy[np.invert(brown_color_filter)]=0 # removing color rgb
        #depth_image_rect_copy[np.invert(blue_color_filter)]=0 # removing color rgb


        

        ######################## PUBLISHING
        # cv2.imshow('depth_image_rect_copy',depth_image_rect_copy)
        # cv2.waitKey(0)
        # depth_image_rect_copy[predominant_blue_pixels_bool]=0 # removing color rgb
        # depth_image_rect_copy_1=depth_image_rect_copy & 0x0F
        # depth_image_rect_copy_0=depth_image_rect_copy & 0xF0
        # print(depth_image_rect_copy_0)
        # print(depth_image_rect_copy_1)
        # depth_image_rect_copy=np.zeros( (depth_image_rect.height, depth_image_rect.width,2),dtype=np.uint8)
        # depth_image_rect_copy[:,:,0]=np.uint8(depth_image_rect_copy_0)
        # depth_image_rect_copy[:,:,1]=np.uint8(depth_image_rect_copy_1)
        # self.imgmsg_depth = Image()
        self.imgmsg_depth = self.bridge.cv2_to_imgmsg(depth_image_rect_copy, "16UC1")
        self.imgmsg_depth.header = depth_image_rect.header
        self.imgmsg_depth.height=depth_image_rect.height
        self.imgmsg_depth.width=depth_image_rect.width
        self.imgmsg_depth.encoding="16UC1"
        # self.imgmsg_depth.step=len(self.imgmsg_depth.data) // self.imgmsg_depth.height
        self.depth_map_pub.publish(self.imgmsg_depth)
        #
        # self.imgmsg_depth_cables = self.bridge.cv2_to_imgmsg(depth_image_rect_copy_cables, "16UC1")
        # self.imgmsg_depth_cables.header = depth_image_rect.header
        # self.imgmsg_depth_cables.height=depth_image_rect.height
        # self.imgmsg_depth_cables.width=depth_image_rect.width
        # self.imgmsg_depth_cables.encoding="16UC1"
        # # self.imgmsg_depth.step=len(self.imgmsg_depth.data) // self.imgmsg_depth.height
        # self.depth_cables_pub.publish(self.imgmsg_depth_cables)

        # print(np.shape(depth_image_rect_copy))
        # print(type(depth_image_rect_copy))
        # print(depth_image_rect_copy)
        # depth_image_rect_copy=depth_image_rect_copy.reshape(depth_image_rect.height, depth_image_rect.width, 2)
        # print(np.shape(depth_image_rect_copy))
        # self.imgmsg_d = Image()
        # self.imgmsg_d.data = depth_image_rect_copy.flatten().tolist()
        # self.imgmsg_d.height=depth_image_rect_copy.shape[0]
        # self.imgmsg_d.width=depth_image_rect_copy.shape[1]
        # self.imgmsg_d.encoding="16UC1"
        # self.imgmsg_d.step=len(self.imgmsg_d.data) // self.imgmsg_d.height

        # self.depth_map_pub.publish(self.imgmsg_d)





    def start(self):
        rospy.loginfo("Starting pcl preprocesser")
        # srv = Server(Reconfig_paramsConfig, self.recon_callback)
        # r = rospy.Rate(10)
        color_image_rect = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_image_rect = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ts = message_filters.TimeSynchronizer([color_image_rect, depth_image_rect], 10)

        ts.registerCallback(self.reading_callback)
        rospy.loginfo("pointcloud_preprocess_is_working")
        # rospy.logerr(sys.version)


        rospy.spin()
        # array_msg=Float64MultiArray()
        # array_msg.data=self.crop_box
        # self.crop_par.publish(array_msg)
        # r.sleep()

if __name__ == '__main__':
	rospy.init_node('pcl_preprocess', anonymous=True)
	#rospy.Subscriber("/usb_cam/image_rect_color",Image,image_callback)

	my_node = pcl_preprocesser()
	my_node.start()
