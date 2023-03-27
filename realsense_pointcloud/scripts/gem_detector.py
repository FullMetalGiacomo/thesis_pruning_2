#!/usr/bin/env python3

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

class gem_detector(object):
    def __init__(self):
        # Params
        # self.known_w = 0.5 #In cm
        # self.cam_inf=rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        # self.focal_length=self.cam_inf.K[0]
        self.cropParam=[0.2,0.4,0.2,0.2] #top, right, bottom, left
        self.crop_box=[0.0,0.0]
        self.radius=50
        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(10)
        self.bridge = CvBridge()


        # Publishers
        self.rgb_pub = rospy.Publisher('/gems_rgb_image',  Image, queue_size=1)
        self.depth_map_pub = rospy.Publisher('/gems_depth_image',  Image, queue_size=1)
        self.crop_par=rospy.Publisher('/cutting_pose',PoseStamped, queue_size=1)
        # Subscribers


    def reading_callback(self, color_image_rect, depth_image_rect):
        # rospy.loginfo("""Reconfigure Request: {left_crop_param}, {right_crop_param},{top_crop_param}, {bottom_crop_param}""".format(**config))
        ################################################# READING
        color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
        depth_image_rect_copy=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)

        image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2RGB)
        # we get the point from the hand
        self.get_point()
        image_point=np.copy(image)
        image_point = cv2.circle(image_point, (self.u,self.v), radius=3, color=(0, 0, 255), thickness=-1)
        image_point_area = cv2.circle(image_point, (self.u,self.v), radius=self.radius, color=(255, 0, 0), thickness=2)
        cv2.imshow('image',image_point)
        crop_img = image[self.v-self.radius:self.v+self.radius,self.u-self.radius:self.u+self.radius]
        cv2.imshow('crop_img',crop_img)

        # binarizing and segmenting
        r=242
        g=222
        b=130
        crop_img[np.invert((crop_img[:,:,0]<b) & (crop_img[:,:,1]<g) & (crop_img[:,:,2]<r))]=0
        gray_im = cv2.cvtColor(crop_img, cv2.COLOR_RGB2GRAY)
        # ret,im_bin = cv2.threshold(gray_im,1,255,cv2.THRESH_BINARY)
        cv2.imshow('crop_img',crop_img)
        # cv2.imshow('im_bin',im_bin)
        # find Harris corners
        img = np.copy(crop_img)
        gray = np.float32(gray_im)
        dst = cv2.cornerHarris(gray,2,3,0.04)
        cv2.imshow('dst',dst)
        dst = cv2.dilate(dst,None)
        cv2.imshow('dst_2',dst)

        ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
        dst = np.uint8(dst)
        cv2.imshow('dst_3',dst) 
        # find centroids
        ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
        # define the criteria to stop and refine the corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
        # Now draw them
        res = np.hstack((centroids,corners))
        res = np.int0(res)
        img[res[:,1],res[:,0]]=[0,0,255]
        img[res[:,3],res[:,2]] = [0,255,0]

        cv2.imshow('img',img)

        cv2.waitKey(0)



        # print(self.cropParam)
    def get_point(self):
        self.u= 320
        self.v= 390

    def rgb_callback(self, data):
        # image=np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        image = np.fromstring(data.data, np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
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
        rospy.loginfo("gem_detector_is_starting ...")
        # srv = Server(Reconfig_paramsConfig, self.recon_callback)
        # r = rospy.Rate(10)
        color_image_rect = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_image_rect = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        cam_inf=rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
        ts = message_filters.TimeSynchronizer([color_image_rect, depth_image_rect], 10)

        ts.registerCallback(self.reading_callback)
        rospy.loginfo("gem_detector_is_working......")
        # rospy.logerr(sys.version)

        rospy.spin()


if __name__ == '__main__':
	rospy.init_node('gem_detect', anonymous=True)
	#rospy.Subscriber("/usb_cam/image_rect_color",Image,image_callback)

	my_node = gem_detector()
	my_node.start()
