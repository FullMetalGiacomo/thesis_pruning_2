#!/usr/bin/env python3

import sys
import math

import cv2
from matplotlib import pyplot as plt
import rospy
import time
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
import scipy.ndimage as scind

# roslaunch realsense2_camera rs_camera.launch align_depth:=true mode:=manualcolor_fps:=15 color_width:=1280 color_height:=720 depth_fps:=15 depth_width:=1280 depth_height:=720 enable_pointcloud:=True flters:=spatial,temporal,hole_filling,decimation,disparity


class pcl_preprocesser(object):
    def __init__(self):

        self.bridge = CvBridge()
        # Publishers
        self.depth_map_pub = rospy.Publisher('/filtered_depth_image',  Image, queue_size=1)





    def reading_callback(self,color_image_rect, depth_image_rect):
        ################################################# READING
        color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
        depth_image_rect_copy=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)
        depth_image_tests=np.copy(depth_image_rect_copy)
        depth_image_tests_1=np.copy(depth_image_rect_copy)

        image_COL = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2RGB)
        # cv2.imshow('Color image',image_COL)
        depth_image_tests_1[depth_image_tests_1 >1500]=0
        depth_image_tests_1[depth_image_tests_1 <350]=0
        cv_image_norm = cv2.normalize(depth_image_tests_1, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('depth_image',cv_image_norm.astype(np.uint8))
        # cv2.waitKey(0)

        ################################################# PLANT SEGMENTATION

        HLS_image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2HLS_FULL)
        histr = cv2.calcHist(HLS_image[:,:,1], [0], None, [256], (0,255)) # getting histogram of lightness
        #  Probability distribution
        P = histr/sum(histr);
        w = np.cumsum(P);
        # plt.plot(w)
        w[w>0.1]=0 # cutting at 10% lightness
        idx=np.argmax(w)
        # plt.plot(w)
        # plt.show()


        lightness_thresh=int(idx)
        if int(idx)>100:
            lightness_thresh=100

        lightness_thresh=100
        HLS_image_without_lightness = cv2.inRange(HLS_image[:,:,1], lightness_thresh, 255)
        # cv2.imshow('HLS_image_whithout_lightness',HLS_image_without_lightness)
        # cv2.waitKey(0)



        # cv2.waitKey(0)
        #############################3 creation of white masks
        r=242
        g=222
        b=130
        color_image_rect_copy=np.copy(color_image_rect)
        color_image_rect_copy[:,:,0]=color_image_rect[:,:,2]
        color_image_rect_copy[:,:,2]=color_image_rect[:,:,0]
        # cv2.imshow('color_image_rect_copy_color',color_image_rect_copy)

        color_image_rect_copy[np.invert((color_image_rect_copy[:,:,0]<r) & (color_image_rect_copy[:,:,1]<g) & (color_image_rect_copy[:,:,2]<b))]=0
        # cv2.imshow('color_image_rect_copy',color_image_rect_copy)
        gray_im = cv2.cvtColor(color_image_rect_copy, cv2.COLOR_RGB2GRAY)
        ret,im_bin = cv2.threshold(gray_im,1,255,cv2.THRESH_BINARY_INV)
        im_bin_bool_sky=(HLS_image[:,:,1]>lightness_thresh) # sky
        im_bin_bool_plant=(HLS_image[:,:,1]<lightness_thresh) # sky
        im_bin=np.zeros(shape=(720,1280))
        im_bin[im_bin_bool_plant]=255
        # cv2.imshow('im_bin_bool_sky',im_bin)
        # cv2.waitKey(0)



        ##############3 depth processing ######################### https://docs.scipy.org/doc/scipy-0.14.0/reference/ndimage.html

        depth_image_tests[depth_image_tests >1500]=0
        depth_image_tests[depth_image_tests <350]=0


        kernel_dil=(3,3)
        kernel_erosion=(3,3)
        kernel_blur=(11,11)
        # blur on "normal image"
        grey_dil=scind.grey_dilation(depth_image_tests,kernel_dil)
        processed_depth_image=scind.grey_erosion(grey_dil,kernel_erosion)
        processed_depth_image[im_bin_bool_sky]=0
        # cv_image_norm = cv2.normalize(processed_depth_image, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('processed_depth_image',cv_image_norm.astype(np.uint8))

        # grey_dil_1=scind.grey_dilation(depth_image_tests,kernel_dil)
        # grey_dil_1=scind.grey_erosion(grey_dil_1,kernel_erosion)
        # grey_dil_1[im_bin_bool_sky]=0
        # grey_dil_1 = cv2.GaussianBlur(grey_dil_1, kernel_blur,1)
        # cv_image_norm = cv2.normalize(grey_dil_1, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('grey_dil_1',cv_image_norm.astype(np.uint8))





        # cv_image_norm = cv2.normalize(processed_depth_image, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('processed_depth_image',cv_image_norm.astype(np.uint8))
        # cv2.waitKey(0)
        ######################## PUBLISHING
        self.imgmsg_depth = self.bridge.cv2_to_imgmsg(processed_depth_image, "16UC1")
        self.imgmsg_depth.header = depth_image_rect.header
        self.imgmsg_depth.height=depth_image_rect.height
        self.imgmsg_depth.width=depth_image_rect.width
        self.imgmsg_depth.encoding="16UC1"
        # self.imgmsg_depth.step=len(self.imgmsg_depth.data) // self.imgmsg_depth.height
        self.depth_map_pub.publish(self.imgmsg_depth)






    def start(self):
        rospy.loginfo("Starting pcl preprocesser")
        color_image_rect = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_image_rect = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ts = message_filters.TimeSynchronizer([color_image_rect, depth_image_rect], 10)

        ts.registerCallback(self.reading_callback)
        rospy.loginfo("pointcloud_preprocess_is_working")
        rospy.spin()


if __name__ == '__main__':
	rospy.init_node('pcl_preprocess', anonymous=True)
	#rospy.Subscriber("/usb_cam/image_rect_color",Image,image_callback)

	my_node = pcl_preprocesser()
	my_node.start()
