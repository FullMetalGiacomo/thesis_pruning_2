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

from skspatial.objects import Plane, Points
from skspatial.plotting import plot_3d

# roslaunch realsense2_camera rs_camera.launch align_depth:=true mode:=manualcolor_fps:=15 color_width:=1280 color_height:=720 depth_fps:=15 depth_width:=1280 depth_height:=720 enable_pointcloud:=True flters:=spatial,temporal,hole_filling,decimation,disparity

class cable_preprocesser(object):
    def __init__(self):
        # Params
        # self.known_w = 0.5 #In cm
        # self.cam_inf=rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        # self.focal_length=self.cam_inf.K[0]
        # self.cropParam=[0.2,0.4,0.2,0.2] #top, right, bottom, left
        # self.crop_box=[0.0,0.0]
        self.u=np.arange(0,1280,dtype=int)
        self.u=np.resize(self.u,(720,1280))
        self.v=np.array(np.arange(0,720))
        self.v=np.resize(self.v,(1280,720)).T

        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(10)
        self.bridge = CvBridge()
        # Publishers
        # self.rgb_pub = rospy.Publisher('/rgb_crop',  Image, queue_size=1)
        self.depth_cables_pub = rospy.Publisher('/filtered_depth_cables',  Image, queue_size=1)
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


        start_time = time.time()

        color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
        depth_image_rect_copy=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)
        depth_image_rect_copy_cables=np.copy(depth_image_rect_copy)

        HLS_image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2HLS_FULL)

        ################################################# CABLES SEGMENTATION https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        lightness_thresh_cables=150
        depth_image_rect_copy_cables[HLS_image[:,:,1]>lightness_thresh_cables]=0

        ################ image processing to get best thinned image
        # ret,gray_im = cv2.threshold(color_image_rect,127,255,cv2.THRESH_BINARY)
        # cv2.imshow('color_image_rect',color_image_rect)
        gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
        # cv2.imshow('gray_im',gray_im)
        # cv2.waitKey(0)
        blur_im = cv2.GaussianBlur(gray_im, (3,3),1)
        # cv2.imshow('blur_im',blur_im)
        # cv2.waitKey(0)
        # canny_im = cv2.Canny(blur_im, 50, 150)
        # cv2.imshow('canny_im',canny_im)
        # cv2.waitKey(0)

        binarized_im = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,5,3)
        # cv2.imshow('binarized_im',binarized_im)
        # kernel = np.ones((3,3),np.uint8)
        # closed_im = cv2.morphologyEx(binarized_im1, cv2.MORPH_CLOSE, kernel)
        # cv2.imshow('closed_im',closed_im)
        # cv2.waitKey(0)
        # binarized_im2 = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,5,2)
        # cv2.imshow('binarized_im2',binarized_im2)
        # cv2.waitKey(0)

        # thinned1 = cv2.ximgproc.thinning(binarized_im1, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        # cv2.imshow('thinned_bin_1_ghuoall',thinned1)
        thinned_im = cv2.ximgproc.thinning(binarized_im)
        # cv2.imshow('thinned_im',thinned_im)
        # cv2.waitKey(0)
        # border_im = cv2.copyMakeBorder(thinned2, top=15, bottom=15, left=15, right=15, borderType=cv2.BORDER_CONSTANT)
        # cv2.imshow('border_im',border_im)
        # thinned11 = cv2.ximgproc.thinning(binarized_im2, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        # cv2.imshow('thinned_bin_2_gouall',thinned11)
        # thinned22 = cv2.ximgproc.thinning(binarized_im2)
        # cv2.imshow('thinned_bin_2',thinned22)
        # cv2.waitKey(0)


        # Hough transform copies of thinned image for show
        cdst = cv2.cvtColor(thinned_im, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)
        # Hough values on rosbags
        hough_tresh=150
        minLineLength_value=150
        maxLineGap_value =10
        #########################3 HOUGH transform raw plot
        # linesP = cv2.HoughLinesP(thinned_im, 1, np.pi / 180, hough_tresh, None,minLineLength=minLineLength_value,maxLineGap=maxLineGap_value)
        # if linesP is not None:
        #     for i in range(0, len(linesP)):
        #         l = linesP[i][0]
        #         cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)
        # cv2.imshow("gray_im", gray_im)
        # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        # cv2.waitKey(0)

        # the idea is to count the slope of the lines
        try:
            linesP = cv2.HoughLinesP(thinned_im, 1, np.pi / 180, hough_tresh, None,minLineLength=minLineLength_value,maxLineGap=maxLineGap_value)
            angle_list=[]
            b_list=[]
            if linesP is not None:
                for i in range(0, len(linesP)):
                    # rospy.loginfo(linesP)
                    # rospy.loginfo(type(linesP))
                    # rospy.loginfo(linesP.shape)
                    l = linesP[i][0]
                    angle = np.arctan((l[3]-l[1]) /(l[2]-l[0])) *180/np.pi
                    # b_list.append(((-l[1]+720)-slope*l[0])/100)
                    b_list.append(l[1]- ((l[0]/(l[2]-l[0]))*(l[3]-l[1])))
                    angle_list.append(angle)
                    r = np.random.randint(256)
                    g = np.random.randint(256)
                    b = np.random.randint(256)
                    cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (r,g,b), 2, cv2.LINE_AA)

            # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
            # cv2.waitKey(0)
            angle_list = np.round(np.array(angle_list),1)
            b_list = np.array(b_list)
            lines_list = np.round(np.array([b_list,angle_list]),1)

            #normalizing list for data analysis, probably unuseful
            # b_list_norm = (b_list-np.min(b_list))/(np.max(b_list)-np.min(b_list))
            # angle_list_norm = (angle_list-np.min(angle_list))/(np.max(angle_list)-np.min(angle_list))
            # lines_list_norm = np.round(np.array([np.array(b_list_norm),np.array(angle_list_norm)]),2)
            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            #### removing outliers and 0 or 90 degrees lines
            # removing zeros and 90 degrees
            # zeros_index = np.squeeze(np.array(np.where(lines_list[1,:] == 0))) # find idx of zero slope
            # ninety_index = np.squeeze(np.array(np.where(lines_list[1,:] == 90))) # find idx of 90 slope
            #
            # if zeros_index is not None:
            #     lines_list = np.delete(lines_list, zeros_index,1)
            #
            # if ninety_index is not None:
            #     lines_list = np.delete(lines_list, ninety_index,1)

            ################# at the end no classification was made. Ask Ivan.

            # angle_distance_matrix = np.triu(np.round(np.abs(angle_list.T[:, None] - angle_list[None, :]),1))
            # rospy.loginfo(angle_distance_matrix)
            # ninety_degree_lines_idxs = np.where(((angle_distance_matrix<91) & (angle_distance_matrix>70)))
            # rospy.loginfo(ninety_degree_lines_idxs)

            # ninety_degree_lines_idx=np.unique(ninety_degree_lines_idxs[0])
            # rospy.loginfo(ninety_degree_lines_idx)

            # chosen_lines = lines_list[:,ninety_degree_lines_idx] # Uncomment for classification
            chosen_lines = lines_list[:,:]
            # rospy.loginfo(chosen_lines)


            chosen_lines_im_thinned = cv2.cvtColor(thinned_im, cv2.COLOR_GRAY2BGR)
            only_cables_image = np.zeros((depth_image_rect.height,depth_image_rect.width,3), np.uint8)


            if chosen_lines is not None:
                for i in range(0, len(chosen_lines[0])):
                    r = np.random.randint(256)
                    g = np.random.randint(256)
                    b = np.random.randint(256)
                    u1 = 0
                    v1 = int(chosen_lines[0,i])
                    u2 = 1280
                    v2 = int(u2*np.tan(chosen_lines[1,i]*np.pi/180)+chosen_lines[0,i])
                    cv2.line(chosen_lines_im_thinned, (u1, v1), (u2, v2), (r,g,b), 2, cv2.LINE_AA)
                    cv2.line(only_cables_image, (u1, v1), (u2, v2), (255,255,255), 2, cv2.LINE_AA)



            # checkpoint_hough = str((time.time() - start_time)) # 0.1
            # rospy.loginfo("all hough transforms times:")
            # rospy.loginfo(checkpoint_hough)

            ##########################3 PLT PLOT
            # Set the figure size
            # plt.figure("data")
            # plt.scatter(lines_list[0,:],lines_list[1,:])
            # plt.xlabel('b')
            # plt.ylabel('angle')
            # plt.grid()

            # plt.figure("normalized_data")
            # plt.scatter(lines_list_norm[0,:],lines_list_norm[1,:])
            # plt.xlabel('b_normalized')
            # plt.ylabel('angle_normalized')
            # plt.grid()
            # plt.figure("image_reference")
            # plt.imshow(cdstP)
            # plt.figure("image_reference_chosen_lines")
            # plt.imshow(chosen_lines_im_thinned)


            # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
            # rospy.loginfo("histogram part")
            # plt.figure("histogram")
            # angle_list_sorted=np.sort(angle_list)
            # hist,bins = np.histogram(angle_list_sorted)
            # rospy.loginfo(hist)
            # rospy.loginfo(bins)
            # rospy.loginfo(angle_list_sorted)
            # plt.bars(hist, bins)

            # plt.show()

            ############### Only cables processing
            only_cables_gray_image = cv2.cvtColor(only_cables_image, cv2.COLOR_RGB2GRAY)
            th, binarized_only_cables_im = cv2.threshold(only_cables_gray_image, 128, 255, cv2.THRESH_BINARY_INV)
            th, binarized_only_cables_im_plane = cv2.threshold(only_cables_gray_image, 128, 255, cv2.THRESH_BINARY)
            cv2.imshow("binarized_only_cables_im", binarized_only_cables_im)
            # print(type(binarized_only_cables_im))
            binarized_only_cables_im.dtype='bool'
            # rospy.loginfo(binarized_only_cables_im)
            # rospy.loginfo(type(binarized_only_cables_im))
            # rospy.loginfo(binarized_only_cables_im.shape)
            # rospy.loginfo(type(depth_image_rect_copy_cables))
            # rospy.loginfo(depth_image_rect_copy_cables.shape)
            # cv2.waitKey(0)

            depth_image_rect_copy_cables[binarized_only_cables_im]=0
            # rospy.loginfo(type(depth_image_rect_copy_cables))
            # rospy.loginfo(depth_image_rect_copy_cables.shape)

            ##############3 i need to create a plane on which the lines are staying.
            ################# select only close distance points

            plane_points_generators=np.copy(depth_image_rect_copy_cables)
            plane_points_generators[plane_points_generators[:,:] < 400]=0
            plane_points_generators[plane_points_generators[:,:] > 1200]=0 # its millimiters!
            # print(plane_points_generators.mean())
            # print(plane_points_generators.shape)
            # print(np.count_nonzero(plane_points_generators))
            # print(type(plane_points_generators))

            # checkpoint_cleaning = str((time.time() - start_time)) # 0.1
            # rospy.loginfo("all cleaning  times:")
            # rospy.loginfo(checkpoint_cleaning)

            plane_thresh_im=np.copy(plane_points_generators)

            #fake depth image for plane creation

            fake_image = np.zeros(shape=(720,1280,3))
            fake_image[:,:,0]=self.u
            fake_image[:,:,1]=self.v
            fake_image[:,:,2]= plane_points_generators
            fake_image_vector=np.reshape(fake_image,(1280*720,3))

            # fake_image_vector = fake_image_vector[np.all(fake_image_vector[:,2] != 0, axis=1)]
            fake_image_vector = fake_image_vector[(fake_image_vector[:,2] != 0)] #removing depth rows =0

            # checkpoint_fake_img = str((time.time() - start_time))
            # rospy.loginfo("checkpoint_fake_img:")
            # rospy.loginfo(checkpoint_fake_img)

            # randomly reducing vector size
            remove_idx= np.random.randint(0,fake_image_vector.shape[0],int(fake_image_vector.shape[0]*0.5)) # removing 0.n% of data
            fake_image_vector=np.delete(fake_image_vector, remove_idx, axis=0)
            #clearing distance with percentile
            # number_of_points=fake_image_vector.shape[0]
            # rospy.logerr(fake_image_vector)
            # rospy.logerr(fake_image_vector.shape)
            # mediana_z_clean = np.median(fake_image_reco_vector[:,2],)
            clear_idx= ((fake_image_vector[:,2]>np.percentile(fake_image_vector[:,2],30)) & (fake_image_vector[:,2]<np.percentile(fake_image_vector[:,2],70)))
            # rospy.logerr(clear_idx)
            # rospy.logerr(clear_idx.shape)
            fake_image_vector=fake_image_vector[clear_idx,:]
            # rospy.logerr(fake_image_vector)
            # rospy.logerr(fake_image_vector.shape)
            # creating plane
            points = Points(fake_image_vector)
            plane = Plane.best_fit(points)
            # rospy.logerr(plane)
            # plot_3d(
            #     points.plotter(c='k', s=1, depthshade=False),
            #     plane.plotter(alpha=0.2, lims_x=(-500, 1000), lims_y=(-500, 1000)
            #     ))
            # plt.show()

            # checkpoint_plane_fit = str((time.time() - start_time))
            # rospy.loginfo("checkpoint_plane_fit:")
            # rospy.loginfo(checkpoint_plane_fit)

            #fake image for cable reconstruction
            fake_image_reco = np.zeros(shape=(720,1280,4))
            fake_image_reco[:,:,0]=self.u
            fake_image_reco[:,:,1]=self.v
            fake_image_reco[:,:,2]= binarized_only_cables_im_plane # this are 255 values
            fake_image_reco[:,:,3]=1
            fake_image_reco_vector=np.reshape(fake_image_reco,(1280*720,4))
            ordering_vector_idx=np.arange(0,fake_image_reco_vector.shape[0]) # used later for reconstructing image
            fake_image_reco_vector[:,3]= ordering_vector_idx

            fake_image_reco_vector_clear = fake_image_reco_vector[(fake_image_reco_vector[:,2] != 0)] # removes depth rows with 0


            # z = z1 + (a(x-x1)+b(y-y1))/c Adding new depths
            x1 = plane.point[0]
            y1 = plane.point[1]
            z1 = plane.point[2]
            a = plane.normal[0]
            b = plane.normal[1]
            c = plane.normal[2]
            rospy.logerr(plane)
            new_z_cables = z1 -(a*(fake_image_reco_vector_clear[:,0]-x1)+b*(fake_image_reco_vector_clear[:,1]-y1))/c
            fake_image_reco_vector_clear[:,2]=new_z_cables



            # removing values that have been changed from fake image
            fake_image_reco_vector=np.delete(fake_image_reco_vector,fake_image_reco_vector_clear[:,3].astype('int'),axis=0)
            # adding new vector
            fake_image_reco_vector=np.concatenate((fake_image_reco_vector, fake_image_reco_vector_clear), axis=0) # adding cables on plane
            # ordering per idx
            fake_image_reco_vector=fake_image_reco_vector[fake_image_reco_vector[:, 3].argsort()] # sorting by index
            fake_image_reco=np.reshape(fake_image_reco_vector,(720,1280,4))# reshaping back into fake image
            cables_on_plane_depth= np.round(fake_image_reco[:,:,2].astype('uint16'))
            #
            # fig = plt.figure(num=1)
            # ax = fig.add_subplot(projection='3d')
            # # ax.set_xlabel('X Label')
            # # ax.set_ylabel('Y Label')
            # # ax.set_zlabel('Z Label')
            # # remove_idx= np.random.randint(0,fake_image_reco_vector_clear.shape[0],int(fake_image_reco_vector_clear.shape[0]*0.999)) # removing 0.n% of data
            # # fake_image_reco_vector_clear=np.delete(fake_image_reco_vector_clear, remove_idx, axis=0)
            # # ax.scatter(fake_image_reco_vector_clear[:,0], fake_image_reco_vector_clear[:,1], fake_image_reco_vector_clear[:,2],s=0.2)
            #
            # plot_points=fake_image_reco_vector[:,[0,1,2]]
            # new_z=np.reshape(cables_on_plane_depth,(1280*720,))
            # plot_points[:,2]=new_z
            # remove_idx= np.random.randint(0,plot_points.shape[0],int(plot_points.shape[0]*0.9999)) # removing 0.n% of data
            # plot_points=np.delete(plot_points, remove_idx, axis=0)
            # # new_z= np.reshape(cables_on_plane_depth,(1280*720,1))
            # points = Points(plot_points)
            # points.plot_3d(ax,c='k', s=0.01, depthshade=False)
            # plane.plot_3d(ax,alpha=0.2, lims_x=(-1000, 1000), lims_y=(-1000, 1000))
            # plt.pause(0.001)
        except:
            rospy.logwarn("no cables detected!")
            fake_image_reco = np.zeros(shape=(720,1280))
            cables_on_plane_depth= np.round(fake_image_reco.astype('uint16'))

        #
        # checkpoint_planes = str((time.time() - start_time))
        # rospy.loginfo("all planes  times:")
        # rospy.loginfo(checkpoint_planes)
        # cables_on_plane_depth_im = cv2.normalize(cables_on_plane_depth, None, 0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        # cv2.imshow('cables_on_plane_depth_im',cables_on_plane_depth_im)
        # cv2.waitKey(0)
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
        # self.imgmsg_depth = self.bridge.cv2_to_imgmsg(depth_image_rect_copy, "16UC1")
        # self.imgmsg_depth.header = depth_image_rect.header
        # self.imgmsg_depth.height=depth_image_rect.height
        # self.imgmsg_depth.width=depth_image_rect.width
        # self.imgmsg_depth.encoding="16UC1"
        # # self.imgmsg_depth.step=len(self.imgmsg_depth.data) // self.imgmsg_depth.height
        # self.depth_map_pub.publish(self.imgmsg_depth)
        #
        # rospy.loginfo("before publishing")
        self.imgmsg_depth_cables = self.bridge.cv2_to_imgmsg(cables_on_plane_depth, "16UC1")
        self.imgmsg_depth_cables.header = depth_image_rect.header
        self.imgmsg_depth_cables.height=depth_image_rect.height
        self.imgmsg_depth_cables.width=depth_image_rect.width
        self.imgmsg_depth_cables.encoding="16UC1"
        # self.imgmsg_depth.step=len(self.imgmsg_depth.data) // self.imgmsg_depth.height
        self.depth_cables_pub.publish(self.imgmsg_depth_cables)



        # checkpoint_end = str((time.time() - start_time))
        # rospy.loginfo("time:")
        # rospy.loginfo(checkpoint_end)

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
        rospy.loginfo("Starting cable preprocesser")
        # srv = Server(Reconfig_paramsConfig, self.recon_callback)
        # r = rospy.Rate(10)
        color_image_rect = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_image_rect = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ts = message_filters.TimeSynchronizer([color_image_rect, depth_image_rect], 10)

        ts.registerCallback(self.reading_callback)
        rospy.loginfo("cables_preprocess_is_working")
        # rospy.logerr(sys.version)


        rospy.spin()
        # array_msg=Float64MultiArray()
        # array_msg.data=self.crop_box
        # self.crop_par.publish(array_msg)
        # r.sleep()

if __name__ == '__main__':
	rospy.init_node('cable_preprocess', anonymous=True)
	#rospy.Subscriber("/usb_cam/image_rect_color",Image,image_callback)

	my_node = cable_preprocesser()
	my_node.start()
