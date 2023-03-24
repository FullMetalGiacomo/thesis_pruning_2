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
import scipy.ndimage as scind

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
        depth_image_tests=np.copy(depth_image_rect_copy)

        # cv2.imshow('color_image_rect',color_image_rect)
        # cv2.waitKey(0)
        ################################################# PLANT SEGMENTATION
        # light removal and experiments on adaptive light threshold ( gone bad because of infrared )
        HLS_image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2HLS_FULL)


        #
        # gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        # # cv2.imshow('gray_im',gray_im)
        # # cv2.waitKey(0)
        # blur_im = cv2.GaussianBlur(gray_im, (3,3),1)
        # # cv2.imshow('blur_im',blur_im)
        # (T,binarized_im_1)  = cv2.threshold(HLS_image[:,:,1], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_TRIANGLE)
        #
        # cv2.imshow(str(T),binarized_im_1)

        # lightness_thresh=0.85*T;
        lightness_thresh=100;
        # HSL_MinLight = np.array([0,  0, 0],np.uint8)
        # HSL_MaxLight = np.array([255, lightness_thresh, 255],np.uint8)
        # cv2.imshow('HLS_image',HLS_image)
        # cv2.waitKey(0)
        HLS_image_without_lightness = cv2.inRange(HLS_image[:,:,1], lightness_thresh, 255)
        cv2.imshow('HLS_image_whithout_lightness',HLS_image_without_lightness)
        # cv2.waitKey(0)
        #############################3 check on brown color filters
        r=242
        g=222
        b=130
        # b=120
        ## rgb rbg bgr brg gbr grb
        # rgb
        brown_color_filter_1=((color_image_rect[:,:,0]<r) & (color_image_rect[:,:,1]<g) & (color_image_rect[:,:,2]<b))
        gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        gray_im[np.invert(brown_color_filter_1)]=0
        cv2.imshow('brown_color_filter_1',gray_im)
        # rbg
        brown_color_filter_2=((color_image_rect[:,:,0]<r) & (color_image_rect[:,:,1]<g) & (color_image_rect[:,:,2]<b))
        gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        gray_im[np.invert(brown_color_filter_2)]=0
        cv2.imshow('brown_color_filter_2',gray_im)
        # bgr
        brown_color_filter_3=((color_image_rect[:,:,0]<g) & (color_image_rect[:,:,1]<b) & (color_image_rect[:,:,2]<r))
        gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        gray_im[np.invert(brown_color_filter_3)]=0
        cv2.imshow('brown_color_filter_3',gray_im)
        # brg
        brown_color_filter_4=((color_image_rect[:,:,0]<g) & (color_image_rect[:,:,1]<r) & (color_image_rect[:,:,2]<b))
        gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        gray_im[np.invert(brown_color_filter_4)]=0
        cv2.imshow('brown_color_filter_4',gray_im)
        # gbr
        brown_color_filter_5=((color_image_rect[:,:,0]<b) & (color_image_rect[:,:,1]<g) & (color_image_rect[:,:,2]<r))

        # grb
        brown_color_filter_6=((color_image_rect[:,:,0]<b) & (color_image_rect[:,:,1]<r) & (color_image_rect[:,:,2]<g))


        brown_color_filter= (brown_color_filter_1) & (brown_color_filter_2) & brown_color_filter_3 & brown_color_filter_4 & brown_color_filter_5 & brown_color_filter_6
        gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        blur_im = cv2.GaussianBlur(gray_im, (3,3),1)
        blur_im[np.invert(brown_color_filter)]=0
        cv2.imshow('whole_filter',blur_im)
        cv2.waitKey(0)

        ########### blue?

        #blue_color_filter=((color_image_rect[:,:,0]<60) & (color_image_rect[:,:,1]<150) & (color_image_rect[:,:,2]<255)) # selecting blues


        # cv2.waitKey(0)
        #
        # gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        # # cv2.imshow('gray_im',gray_im)
        # # cv2.waitKey(0)
        # blur_im = cv2.GaussianBlur(gray_im, (3,3),1)
        # # cv2.imshow('blur_im',blur_im)
        # # cv2.waitKey(0)
        # # canny_im = cv2.Canny(blur_im, 50, 150)
        # # cv2.imshow('canny_im',canny_im)
        # # cv2.waitKey(0)
        #
        # binarized_im = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,13,5)
        # cv2.imshow('binarized_im',binarized_im)
        #
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

        ##############3 tests on depth ######################### https://docs.scipy.org/doc/scipy-0.14.0/reference/ndimage.html
         # the idea is to do a dilation prioritizing the closest branches, therefore we must pass in inverse world.
         # to avoid the background to be the max value we place them at -10 meters when we do morphological operations.
         # When we want to visualize we need to convert back!
        depth_image_tests[depth_image_tests >1500]=0
        depth_image_tests[depth_image_tests <350]=0


        kernel_dil=(9,9)
        kernel_erosion=(5,5)
        kernel_blur=(11,11)
        # blur on "normal image"
        blur_im = cv2.GaussianBlur(depth_image_tests, (11,11),1)
        # cv_image_norm = cv2.normalize(blur_im, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('blur_im',cv_image_norm.astype(np.uint8))

        # blur_im[blur_im >1500]=10000
        # blur_im[blur_im <350]=10000
        # blur_im=-1*blur_im

        # grey closing prioritizing closest plants
        # grey_clos=scind.grey_closing(blur_im,kernel)
        # cv_image_norm = cv2.normalize(grey_clos, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('grey_clos',cv_image_norm.astype(np.uint8))


        # three dilations for improving plant connections prioritizing closest plants
        # grey_dil_1=scind.grey_dilation(grey_clos,kernel)
        grey_dil_1=scind.grey_dilation(depth_image_tests,kernel_dil)
        grey_dil_1=scind.grey_erosion(grey_dil_1,kernel_erosion)
        grey_dil_1[HLS_image[:,:,1]>lightness_thresh]=0
        # grey_dil_1 = cv2.GaussianBlur(grey_dil_1, kernel_blur,1)
        # cv_image_norm = cv2.normalize(grey_dil_1, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('grey_dil_1',cv_image_norm.astype(np.uint8))


        grey_dil_2=scind.grey_dilation(grey_dil_1,kernel_dil)
        grey_dil_2=scind.grey_erosion(grey_dil_2,kernel_erosion)
        grey_dil_2[HLS_image[:,:,1]>lightness_thresh]=0
        # grey_dil_2 = cv2.GaussianBlur(grey_dil_2, kernel_blur,1)
        # cv_image_norm = cv2.normalize(grey_dil_2, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('grey_dil_2',cv_image_norm.astype(np.uint8))

        grey_dil_3=scind.grey_dilation(grey_dil_2,kernel_dil)
        grey_dil_3=scind.grey_erosion(grey_dil_3,kernel_erosion)
        grey_dil_3[HLS_image[:,:,1]>lightness_thresh]=0
        # grey_dil_3 = cv2.GaussianBlur(grey_dil_3, kernel_blur,1)
        # cv_image_norm = cv2.normalize(grey_dil_3, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('grey_dil_3',cv_image_norm.astype(np.uint8))

        grey_dil_4=scind.grey_dilation(grey_dil_3,kernel_dil)
        grey_dil_4=scind.grey_erosion(grey_dil_4,kernel_erosion)
        grey_dil_4[HLS_image[:,:,1]>lightness_thresh]=0
        # grey_dil_4 = cv2.GaussianBlur(grey_dil_4, kernel_blur,1)
        # cv_image_norm = cv2.normalize(grey_dil_4, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow('grey_dil_4',cv_image_norm.astype(np.uint8))
#########################################3 creation of mask plant from color images









        #depth_image_rect_copy[np.invert(HLS_image_without_lightness_bool)]=0 # removing lightness points

        depth_image_rect_copy=np.copy(depth_image_rect_copy)
        # tries to keep the brown colors
        brown_color_filter=((color_image_rect[:,:,0]<242) & (color_image_rect[:,:,1]<222) & (color_image_rect[:,:,2]<202)) # selecting browns
        # reduces effect of infrared
        #blue_color_filter=((color_image_rect[:,:,0]<60) & (color_image_rect[:,:,1]<150) & (color_image_rect[:,:,2]<255)) # selecting blues

        depth_image_rect_copy[HLS_image[:,:,1]>lightness_thresh]=0 # removing lightness points
        # depth_image_rect_copy[np.invert(brown_color_filter)]=0 # removing color rgb
        # depth_image_rect_copy[np.invert(blue_color_filter)]=0 # removing color rgb

        #################33 Testing grey_dil_4
        depth_image_rect_copy = np.copy(grey_dil_4)
        # depth_image_rect_copy[HLS_image[:,:,1]>lightness_thresh]=0



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
