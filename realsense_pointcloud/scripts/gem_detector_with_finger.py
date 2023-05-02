#!/usr/bin/env python3

import sys
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from dynamic_reconfigure.server import Server
from realsense_pointcloud.cfg import Reconfig_paramsConfig


import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import scipy.ndimage as scind
from skimage.morphology import skeletonize

import matplotlib.pyplot as plt
from wand.image import Image as ImgWand
import itertools


class gem_detector(object):
    def __init__(self):
        # radius of the image window in pixels
        self.radius=80
        # initializing cvbridge
        self.bridge = CvBridge()
        self.finger_pose_msg=PoseStamped()
        # Publishers
        self.rgb_pub = rospy.Publisher('/gems_rgb_image',  Image, queue_size=1)
        self.depth_map_pub = rospy.Publisher('/gems_depth_image',  Image, queue_size=1)
        self.pruning_point=rospy.Publisher('/pruning_point_pose',PoseStamped, queue_size=1)

    def get_point(self):
        ## transforming finger pose into pixel!
        x = self.finger_pose_msg.pose.position.x
        y = self.finger_pose_msg.pose.position.y
        z = self.finger_pose_msg.pose.position.z

        self.u= int(x*self.K[0]/z +self.K[2])
        self.v= int(y*self.K[4]/z +self.K[5])

        ## rosbag 0
        #red band
        # self.u= 1080
        # self.v= 270
        #other point
        # self.u= 800
        # self.v= 130



    def reading_callback(self,color_image_rect, depth_image_rect):
        ################################################# READING
        img_header=color_image_rect.header
        color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
        depth_image_rect=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)
        depth_image_rect_copy=np.copy(depth_image_rect)
        color_image_rect_copy=np.copy(color_image_rect)
        depth_image_rect_copy[depth_image_rect_copy >1500]=0
        depth_image_rect_copy[depth_image_rect_copy <350]=0
        image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2RGB)

        # things to do :
        # 0 get image crop around a radious in image
        # 1 Apply watershed algorithm to isolate the branch
        # 2 find skel of crop img and call the nodes as gems.
        # we get the point from the hand and we crop around the interest point

        cv2.startWindowThread()
        self.get_point() # getting the point from the hand
        image_point=np.copy(image)
        image_point = cv2.circle(image_point, (self.u,self.v), radius=3, color=(0, 0, 255), thickness=-1)
        image_point_area = cv2.circle(image_point, (self.u,self.v), radius=self.radius, color=(255, 0, 0), thickness=2)
        cv2.imshow('image',image_point)
        ##### cropping around the point
        crop_img = image[self.v-self.radius:self.v+self.radius,self.u-self.radius:self.u+self.radius]
        crop_img_visualizer=np.copy(crop_img)
        crop_img_viewer = cv2.resize(crop_img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('crop_img',crop_img_viewer)

        #### binarizing and segmenting
        HLS_image = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HLS_FULL)
        lightness_thresh=100
        crop_img[HLS_image[:,:,1]>lightness_thresh]=0        # Segmenting by light
        gray_im = cv2.cvtColor(crop_img, cv2.COLOR_RGB2GRAY)
        ret,im_bin = cv2.threshold(gray_im,1,255,cv2.THRESH_BINARY)
        ret,im_bin_inv = cv2.threshold(gray_im,1,255,cv2.THRESH_BINARY_INV)
        # crop_img_viewer = cv2.resize(crop_img, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('crop_img_grey',crop_img_viewer)

        ### get also crop from depth image
        crop_img_depth=depth_image_rect_copy[self.v-self.radius:self.v+self.radius,self.u-self.radius:self.u+self.radius]
        # cv_image_norm = cv2.normalize(crop_img_depth, None, 0, 255, cv2.NORM_MINMAX)
        # crop_img_viewer=cv2.resize(cv_image_norm, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('crop_img_depth',crop_img_viewer.astype(np.uint8))
        # crop_img_viewer_1=cv2.resize(im_bin_inv.astype(np.uint8), (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('im_bin_inv',crop_img_viewer_1)
        crop_img_viewer_1=cv2.resize(im_bin.astype(np.uint8), (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('im_bin',crop_img_viewer_1)

        crop_img_depth=scind.grey_dilation(crop_img_depth,(5,5)) # dilating and reapplying mask
        crop_img_depth[im_bin_inv.astype('bool')]=0
        # cv_image_norm = cv2.normalize(crop_img_depth, None, 0, 255, cv2.NORM_MINMAX)
        # crop_img_viewer_1=cv2.resize(cv_image_norm, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('crop_img_depth_masked',crop_img_viewer_1.astype(np.uint8))
        cv2.waitKey(0)

        ######################## now we have both a crop of depth and image
        #  use watershed algorithm to isolate the branch
        ####################### watershed SEGMENTATION
        img=np.copy(crop_img)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,1,255,cv2.THRESH_BINARY)
        #### noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 1)
        # crop_img_viewer=cv2.resize(opening, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('noise removal',crop_img_viewer.astype(np.uint8))
        #### sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=1)
        # crop_img_viewer=cv2.resize(sure_bg, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('sure background area',crop_img_viewer.astype(np.uint8))
        #### Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        # crop_img_viewer=cv2.resize(dist_transform, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow("dist_transform",crop_img_viewer.astype(np.uint8))
        ret, sure_fg = cv2.threshold(dist_transform,0.2*dist_transform.max(),255,0)
        # crop_img_viewer=cv2.resize(sure_fg, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow("Finding sure foreground area",crop_img_viewer.astype(np.uint8))

        #### Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg) # do i have it already ?
        # crop_img_viewer=cv2.resize(unknown, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow("Finding unknown region",crop_img_viewer.astype(np.uint8))
        #### Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)
        # crop_img_viewer=cv2.resize(markers.astype(np.uint8), (500,500), interpolation = cv2.INTER_AREA)
        # crop_img_viewer = cv2.normalize(crop_img_viewer, None, 0, 255, cv2.NORM_MINMAX)
        # cv2.imshow("connected components",crop_img_viewer.astype(np.uint8))

        #### Add one to all labels so that sure background is not 0, but 1
        markers = markers+1
        #### Now, mark the region of unknown with zero
        markers[unknown==255] = 0
        #### apply watershed
        markers = cv2.watershed(img,markers)
        #### we need to find marker with max area.
        number_of_markers=markers.max()+1
        #### Iterate through markers and save result of area of each
        max_holder=[]
        for num in range(2, number_of_markers):
            area=np.count_nonzero(markers==num)
            max_holder.append((area,num))

        #### chosing marker with max area
        max_holder=np.array(max_holder)
        chosen_marker_idx=np.argmax(max_holder[:,0])
        chosen_marker=max_holder[chosen_marker_idx,1]
        #### taking the biggest branch outside borders and background
        branch_mask=((markers==chosen_marker).astype("bool")).astype(np.uint8)
        ret,branch_mask= cv2.threshold(branch_mask,0,255,cv2.THRESH_BINARY)
        # crop_img_viewer=cv2.resize(branch_mask, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow("branch_mask",crop_img_viewer.astype(np.uint8))
        #### underlining borders on image
        img[markers == -1] = [255,0,0]
        crop_img_viewer=cv2.resize(img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("img",crop_img_viewer.astype(np.uint8))
        # cv2.imshow('im_bin',im_bin)

        #### apply mask to image
        color_mask=np.copy(crop_img)
        color_mask[np.invert(branch_mask.astype("bool"))]=np.array([0,0,0])
        grey_mask= np.copy(gray_im)
        grey_mask[np.invert(branch_mask.astype("bool"))]=0
        crop_img_viewer=cv2.resize(color_mask, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("color_mask",crop_img_viewer.astype(np.uint8))
        # crop_img_viewer=cv2.resize(grey_mask, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow("grey_mask",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)

        ######################## now we have the segmentation of the branch of interest
        # 3 Detect the gems, we skeletonize the image and we call a match with the junctions
        # the idea is that the skel of branch will have nodes where the gems are!
        ####################### skeletonizing and detecting the junctions

        #### keletonizaition using scipy
        img = np.copy(color_mask)
        grey_mask=np.copy(grey_mask)


        th,bin_mask=cv2.threshold(grey_mask,0,255,cv2.THRESH_BINARY)
        crop_img_viewer=cv2.resize(bin_mask, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow(" bin_mask_pura",crop_img_viewer.astype(np.uint8))


        #### mask enhancment because gems are very hard to detect

        skeleton = skeletonize(bin_mask.astype('bool')).astype(np.uint8) # using scipy function
        skeleton=np.array(skeleton)*255

        # get rotation of branch
        pixelpointsCV2 = np.array(cv2.findNonZero(skeleton))
        u_vector=pixelpointsCV2[:,0,0]
        v_vector=pixelpointsCV2[:,0,1]
        a, b = np.polyfit(u_vector, v_vector, 1)
        a_deg=-np.arctan(a)*180/np.pi # rotation of branch

        # define closing kernel
        kernel= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,5))
        kernel_1= cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        kernel[kernel==255]=1
        # align closing kerner with branch
        kernel_rotated=scind.rotate(kernel.astype(np.uint8),int(a_deg),reshape=True)

        bin_mask_1= cv2.morphologyEx(bin_mask, cv2.MORPH_CLOSE, kernel_rotated) # closing with elliptical kernel
        crop_img_viewer=cv2.resize(bin_mask_1, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow(" bin_mask_1 closed rotated",crop_img_viewer.astype(np.uint8))

        skeleton_dil=cv2.dilate(skeleton.astype(np.uint8),(3,3),iterations = 1) # dilation for seeing it
        crop_img_viewer=cv2.resize(skeleton_dil, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("skeleton_dil",crop_img_viewer.astype(np.uint8))

        #### using wand to detect the junctions of the skel


        # Find line-junctions using Top-Hat Morphology
        # define the kernels for detecting line junctions
        # https://legacy.imagemagick.org/Usage/morphology/
        lineJunctions = """
        3>:
            1,-,1
            -,1,-
            -,1,-;
        3>:
            -,1,-
            -,1,1
            1,-,-;
        3>:
            1,-,-
            -,1,-
            1,-,1
        """
        # Clone the original image as we are about to destroy it
        with ImgWand.from_array(skeleton) as junctionsImage:
            junctionsImage.morphology(method='hit_and_miss', kernel=lineJunctions)
            junctions=np.copy(np.array(junctionsImage))

        #### we now show the junctions overlapped with the branch  (img[markers == -1] = [255,0,0]) for better visualization
        # skel with points
        junction_gray=cv2.cvtColor(junctions,cv2.COLOR_BGR2GRAY)
        th,junctions_bin=cv2.threshold(junction_gray,0,255,cv2.THRESH_BINARY)
        crop_img_viewer=cv2.resize(junctions_bin, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("junctions_bin",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)
        kernel= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        junctions_dil = cv2.dilate(junctions_bin,kernel,iterations = 2).astype(np.uint8)
        junctions_visualizer=np.zeros(color_mask.shape,np.uint8)
        junctions_visualizer[junctions_dil==255]=np.array([0,0,255])

        skel_visualizer = np.zeros(color_mask.shape,np.uint8)
        skel_visualizer[skeleton==255]=np.array([0,255,255])
        overlapped_1 = cv2.addWeighted(skel_visualizer, 0.5, crop_img_visualizer, 1, 0)
        overlapped_2 = cv2.addWeighted(junctions_visualizer, 1, overlapped_1, 1, 0)

        crop_img_viewer=cv2.resize(overlapped_2, (700,700), interpolation = cv2.INTER_AREA)
        cv2.imshow("overlapped_2",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)

#################### removing redundancies
        junctions_vector = np.array(np.nonzero(junctions_bin))
        plt.plot(junctions_vector[0,:], junctions_vector[1,:], marker="o", markersize=20, markeredgecolor="red", markerfacecolor="green")
        plt.show()
        threshold=(12,12)
        coords = (np.array([junctions_vector[1,:],junctions_vector[0,:]]).T).tolist()
        clean_coords=self.process(coords,threshold) # removes the points close to each other
        clean_coords=np.array(clean_coords)
        plt.plot(clean_coords[:,0], clean_coords[:,1], marker="o", markersize=20, markeredgecolor="red", markerfacecolor="green")
        plt.show()

        overlapped_clean=np.copy(overlapped_1)  # shows new points on image
        for point in clean_coords:
            print(point)
            cv2.circle(overlapped_clean,tuple(point),2,(0,0,255),-1)

        crop_img_viewer=cv2.resize(overlapped_clean, (700,700), interpolation = cv2.INTER_AREA)
        cv2.imshow("overlapped_clean",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)
#####################3 Selecting Middle point!
        ## chose the two points closest to the image center!
        if clean_coords.shape[0]==0 or clean_coords.shape[0]==1:
            rospy.logerr("Not Enough Gems Found !!!!!!!!!!!!")
            rospy.logwarn("Giving Back Hand Point")
            self.publish_finger_point()
            return

        elif clean_coords.shape[0]==2:
            chosen_gems=np.copy(clean_coords)
        else:
            distance_vector=np.sqrt((clean_coords[:,0]-self.radius)*(clean_coords[:,0]-self.radius) + (clean_coords[:,1]-self.radius)*(clean_coords[:,1]-self.radius))
            k = 2 # top2 values
            idx_gems = np.argpartition(distance_vector, k)
            chosen_gems=clean_coords[idx_gems[:k]]




        img = np.copy(color_mask)
        for point in chosen_gems:
            cv2.circle(img,tuple(point),2,(0,0,255),-1)

        crop_img_viewer=cv2.resize(img, (700,700), interpolation = cv2.INTER_AREA)
        cv2.imshow("selected_gems",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)

        ## place cutting point in the middle and on skeleton
        mean_u=int(np.mean(chosen_gems[:,0]))
        mean_v=int(np.mean(chosen_gems[:,1]))
        cutting_point=np.array([mean_u,mean_v])
        # moving cutting point on skel
        corrected_cutting_point=np.squeeze(self.find_nearest_white(skeleton,cutting_point))

        img = np.copy(color_mask)
        cv2.circle(img,tuple(cutting_point),2,(0,0,255),-1)
        crop_img_viewer=cv2.resize(img, (700,700), interpolation = cv2.INTER_AREA)
        cv2.imshow("selected_cutting_point",crop_img_viewer.astype(np.uint8))

        img = np.copy(color_mask)
        cv2.circle(img,tuple(corrected_cutting_point),2,(0,0,255),-1)
        crop_img_viewer=cv2.resize(img, (700,700), interpolation = cv2.INTER_AREA)
        cv2.imshow("selected corrected cutting point",crop_img_viewer.astype(np.uint8))

        cv2.waitKey(0)
#####################3 Build 3D point
        ## bring point into img coordinates
        chosen_gems[:,0]=chosen_gems[:,0]+self.u -self.radius
        chosen_gems[:,1]=chosen_gems[:,1]+self.v-self.radius
        corrected_cutting_point[0]=corrected_cutting_point[0]+self.u -self.radius
        corrected_cutting_point[1]=corrected_cutting_point[1]+self.v-self.radius
        ## find distance of point
        inv_bin_mask=cv2.bitwise_not(bin_mask)
        crop_img_depth[inv_bin_mask.astype('bool')]=0
        trues=(crop_img_depth != 0)
        depth_vector=crop_img_depth[trues]
        mean_dist=depth_vector.mean()/1000
        ## transform in x y

        img = np.copy(color_image_rect)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.circle(img,tuple(chosen_gems[0,:]),2,(0,255,0),3)
        cv2.circle(img,tuple(chosen_gems[1,:]),2,(0,255,0),3)
        cv2.circle(img,tuple(corrected_cutting_point),2,(0,0,255),3)
        cv2.circle(img, (self.u,self.v), radius=self.radius, color=(255, 0, 0), thickness=2)
        cv2.imshow("selected corrected cutting point",img.astype(np.uint8))
        cv2.waitKey(0)
################# Publish point
        x= (mean_dist/self.K[0])*(corrected_cutting_point[0]-self.K[2])
        y= (mean_dist/self.K[4])*(corrected_cutting_point[1]-self.K[5])
        print(a_deg)
        roll = 0
        pitch = 0
        yaw = np.radians(-a_deg)
        [qx, qy, qz, qw] = self.get_quaternion_from_euler(roll,pitch,yaw)
        point=PoseStamped()

        point.header = img_header
        point.pose.orientation.x = qx
        point.pose.orientation.y = qy
        point.pose.orientation.z = qz
        point.pose.orientation.w = qw

        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = mean_dist
        self.pruning_point.publish(point)


    def process(self,input_list, threshold):
        combos = itertools.combinations(input_list, 2)
        points_to_remove = [point2
                            for point1, point2 in combos
                            if abs(point1[0]-point2[0])<=threshold[0] and abs(point1[1]-point2[1])<=threshold[1]]
        points_to_keep = [point for point in input_list if point not in points_to_remove]
        return points_to_keep


    def find_nearest_white(self,img, target):
        nonzero = cv2.findNonZero(img)
        distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
        nearest_index = np.argmin(distances)
        return nonzero[nearest_index]

    def publish_finger_point(self):
        self.pruning_point.publish(self.finger_pose_msg)

    def get_quaternion_from_euler(self,roll, pitch, yaw):

      #Convert an Euler angle to a quaternion.
      qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
      qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
      qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
      qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

      return [qx, qy, qz, qw]


    def start(self):
        rospy.loginfo("gem_detector_is_starting ...")
        rospy.loginfo("Subscribing to /finger_pose topic...")
        self.finger_pose_msg=rospy.wait_for_message("/finger_pose", PoseStamped)
        rospy.loginfo("finger_pose received!")
        color_image_rect = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_image_rect = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        cam_inf=rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
        self.K=cam_inf.K
        ts = message_filters.TimeSynchronizer([color_image_rect, depth_image_rect], 10)

        ts.registerCallback(self.reading_callback)
        rospy.loginfo("gem_detector_is_working......")

        rospy.spin()


if __name__ == '__main__':
	rospy.init_node('gem_detect', anonymous=True)
	#rospy.Subscriber("/usb_cam/image_rect_color",Image,image_callback)

	my_node = gem_detector()
	my_node.start()
