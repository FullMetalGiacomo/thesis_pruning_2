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
        lightness_thresh=255; # dynamic param should be implemented
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


        # depth_image_rect_copy[np.invert(HLS_image_without_lightness_bool)]=0 # removing lightness points

        depth_image_rect_copy=np.copy(depth_image_rect_copy)

        #brown_color_filter=((color_image_rect[:,:,0]<242) & (color_image_rect[:,:,1]<222) & (color_image_rect[:,:,2]<202)) # selecting browns
        # reduces effect of infrared
        #blue_color_filter=((color_image_rect[:,:,0]<60) & (color_image_rect[:,:,1]<150) & (color_image_rect[:,:,2]<255)) # selecting blues

        depth_image_rect_copy[HLS_image[:,:,1]>lightness_thresh]=0 # removing lightness points
        #depth_image_rect_copy[np.invert(brown_color_filter)]=0 # removing color rgb
        #depth_image_rect_copy[np.invert(blue_color_filter)]=0 # removing color rgb


        ################################################# CABLES SEGMENTATION https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        # lightness_thresh_cables=150
        # depth_image_rect_copy_cables[HLS_image[:,:,1]>lightness_thresh_cables]=0
        #
        # ################ image processing to get best thinned image
        # # ret,gray_im = cv2.threshold(color_image_rect,127,255,cv2.THRESH_BINARY)
        # # cv2.imshow('color_image_rect',color_image_rect)
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
        # binarized_im = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,5,3)
        # # cv2.imshow('binarized_im1',binarized_im1)
        # # kernel = np.ones((3,3),np.uint8)
        # # closed_im = cv2.morphologyEx(binarized_im1, cv2.MORPH_CLOSE, kernel)
        # # cv2.imshow('closed_im',closed_im)
        # # cv2.waitKey(0)
        # # binarized_im2 = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,5,2)
        # # cv2.imshow('binarized_im2',binarized_im2)
        # # cv2.waitKey(0)
        #
        # # thinned1 = cv2.ximgproc.thinning(binarized_im1, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        # # cv2.imshow('thinned_bin_1_ghuoall',thinned1)
        # thinned_im = cv2.ximgproc.thinning(binarized_im)
        # # cv2.imshow('thinned_im',thinned_im)
        # # cv2.waitKey(0)
        # # border_im = cv2.copyMakeBorder(thinned2, top=15, bottom=15, left=15, right=15, borderType=cv2.BORDER_CONSTANT)
        # # cv2.imshow('border_im',border_im)
        # # thinned11 = cv2.ximgproc.thinning(binarized_im2, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        # # cv2.imshow('thinned_bin_2_gouall',thinned11)
        # # thinned22 = cv2.ximgproc.thinning(binarized_im2)
        # # cv2.imshow('thinned_bin_2',thinned22)
        # # cv2.waitKey(0)
        #
        #
        # # Hough transform copies of thinned image for show
        # cdst = cv2.cvtColor(thinned_im, cv2.COLOR_GRAY2BGR)
        # cdstP = np.copy(cdst)
        # # Hough values on rosbags
        # hough_tresh=150
        # minLineLength_value=150
        # maxLineGap_value =10
        # #########################3 HOUGH transform raw plot
        # # linesP = cv2.HoughLinesP(thinned_im, 1, np.pi / 180, hough_tresh, None,minLineLength=minLineLength_value,maxLineGap=maxLineGap_value)
        # # if linesP is not None:
        # #     for i in range(0, len(linesP)):
        # #         l = linesP[i][0]
        # #         cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)
        # # cv2.imshow("gray_im", gray_im)
        # # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        # # cv2.waitKey(0)
        #
        # # the idea is to count the slope of the lines
        # linesP = cv2.HoughLinesP(thinned_im, 1, np.pi / 180, hough_tresh, None,minLineLength=minLineLength_value,maxLineGap=maxLineGap_value)
        # angle_list=[]
        # b_list=[]
        # if linesP is not None:
        #     for i in range(0, len(linesP)):
        #         # rospy.loginfo(linesP)
        #         # rospy.loginfo(type(linesP))
        #         # rospy.loginfo(linesP.shape)
        #         l = linesP[i][0]
        #         angle = np.arctan((l[3]-l[1]) /(l[2]-l[0])) *180/np.pi
        #         # b_list.append(((-l[1]+720)-slope*l[0])/100)
        #         b_list.append(l[1]- ((l[0]/(l[2]-l[0]))*(l[3]-l[1])) )
        #         angle_list.append(angle)
        #         r = np.random.randint(256)
        #         g = np.random.randint(256)
        #         b = np.random.randint(256)
        #         cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (r,g,b), 2, cv2.LINE_AA)
        #
        #
        # angle_list = np.round(np.array(angle_list),1)
        # b_list = np.array(b_list)
        # lines_list = np.round(np.array([b_list,angle_list]),1)
        #
        # #normalizing list for data analysis, probably unuseful
        # # b_list_norm = (b_list-np.min(b_list))/(np.max(b_list)-np.min(b_list))
        # # angle_list_norm = (angle_list-np.min(angle_list))/(np.max(angle_list)-np.min(angle_list))
        # # lines_list_norm = np.round(np.array([np.array(b_list_norm),np.array(angle_list_norm)]),2)
        #
        #
        # #### removing outliers and 0 or 90 degrees lines
        # # removing zeros and 90 degrees
        # # zeros_index = np.squeeze(np.array(np.where(lines_list[1,:] == 0))) # find idx of zero slope
        # # ninety_index = np.squeeze(np.array(np.where(lines_list[1,:] == 90))) # find idx of 90 slope
        # #
        # # if zeros_index is not None:
        # #     lines_list = np.delete(lines_list, zeros_index,1)
        # #
        # # if ninety_index is not None:
        # #     lines_list = np.delete(lines_list, ninety_index,1)
        #
        # ################# at the end no classification was made. Ask Ivan.
        #
        # # angle_distance_matrix = np.triu(np.round(np.abs(angle_list.T[:, None] - angle_list[None, :]),1))
        # # rospy.loginfo(angle_distance_matrix)
        # # ninety_degree_lines_idxs = np.where(((angle_distance_matrix<91) & (angle_distance_matrix>70)))
        # # rospy.loginfo(ninety_degree_lines_idxs)
        #
        # # ninety_degree_lines_idx=np.unique(ninety_degree_lines_idxs[0])
        # # rospy.loginfo(ninety_degree_lines_idx)
        #
        # # chosen_lines = lines_list[:,ninety_degree_lines_idx] # Uncomment for classification
        # chosen_lines = lines_list[:,:]
        # # rospy.loginfo(chosen_lines)
        #
        #
        # chosen_lines_im_thinned = cv2.cvtColor(thinned_im, cv2.COLOR_GRAY2BGR)
        # only_cables_image = np.zeros((depth_image_rect.height,depth_image_rect.width,3), np.uint8)
        #
        #
        # if chosen_lines is not None:
        #     for i in range(0, len(chosen_lines[0])):
        #         r = np.random.randint(256)
        #         g = np.random.randint(256)
        #         b = np.random.randint(256)
        #         u1 = 0
        #         v1 = int(chosen_lines[0,i])
        #         u2 = 1280
        #         v2 = int(u2*np.tan(chosen_lines[1,i]*np.pi/180)+chosen_lines[0,i])
        #         cv2.line(chosen_lines_im_thinned, (u1, v1), (u2, v2), (r,g,b), 2, cv2.LINE_AA)
        #         cv2.line(only_cables_image, (u1, v1), (u2, v2), (255,255,255), 1, cv2.LINE_AA)
        #
        #
        #
        #
        #
        # ###########################3 PLT PLOT
        # # Set the figure size
        # # plt.figure("data")
        # # plt.scatter(lines_list[0,:],lines_list[1,:])
        # # plt.xlabel('b')
        # # plt.ylabel('angle')
        # # plt.grid()
        # #
        # # plt.figure("normalized_data")
        # # plt.scatter(lines_list_norm[0,:],lines_list_norm[1,:])
        # # plt.xlabel('b_normalized')
        # # plt.ylabel('angle_normalized')
        # # plt.grid()
        # # plt.figure("image_reference")
        # # plt.imshow(cdstP)
        # # plt.figure("image_reference_chosen_lines")
        # # plt.imshow(chosen_lines_im)
        # #
        # #
        # # # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        # # # rospy.loginfo("histogram part")
        # # # plt.figure("histogram")
        # # # angle_list_sorted=np.sort(angle_list)
        # # # hist,bins = np.histogram(angle_list_sorted)
        # # # rospy.loginfo(hist)
        # # # rospy.loginfo(bins)
        # # # rospy.loginfo(angle_list_sorted)
        # # # plt.bars(hist, bins)
        # #
        # # plt.show()
        #
        # ############### Only cables processing
        # only_cables_gray_image = cv2.cvtColor(only_cables_image, cv2.COLOR_RGB2GRAY)
        # th, binarized_only_cables_im = cv2.threshold(only_cables_gray_image, 128, 255, cv2.THRESH_BINARY_INV)
        # # cv2.imshow("binarized_only_cables_im", binarized_only_cables_im)
        # binarized_only_cables_im.dtype='bool'
        # # rospy.loginfo(binarized_only_cables_im)
        # # rospy.loginfo(type(binarized_only_cables_im))
        # # rospy.loginfo(binarized_only_cables_im.shape)
        # # rospy.loginfo(type(depth_image_rect_copy_cables))
        # # rospy.loginfo(depth_image_rect_copy_cables.shape)
        # # cv2.waitKey(0)
        #
        # depth_image_rect_copy_cables[binarized_only_cables_im]=0
        # # rospy.loginfo(type(depth_image_rect_copy_cables))
        # # rospy.loginfo(depth_image_rect_copy_cables.shape)


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
