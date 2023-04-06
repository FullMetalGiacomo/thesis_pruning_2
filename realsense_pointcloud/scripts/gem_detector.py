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

import scipy.ndimage as scind
from skimage.morphology import skeletonize
import sknw

import matplotlib.pyplot as plt
from wand.image import Image as ImgWand


class gem_detector(object):
    def __init__(self):
        # Params
        # self.known_w = 0.5 #In cm
        # self.cam_inf=rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        # self.focal_length=self.cam_inf.K[0]
        self.cropParam=[0.2,0.4,0.2,0.2] #top, right, bottom, left
        self.crop_box=[0.0,0.0]
        self.radius=80
        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(10)
        self.bridge = CvBridge()


        # Publishers
        self.rgb_pub = rospy.Publisher('/gems_rgb_image',  Image, queue_size=1)
        self.depth_map_pub = rospy.Publisher('/gems_depth_image',  Image, queue_size=1)
        self.crop_par=rospy.Publisher('/cutting_pose',PoseStamped, queue_size=1)
        # Subscribers

    def get_point(self):
        ## rosbag 0
        #red band
        # self.u= 1080
        # self.v= 270
        #other point
        # self.u= 800
        # self.v= 130

        ## rosbag 1
        #cluster
        # self.u= 780
        # self.v= 150
        #other point
        # self.u= 460
        # self.v= 260

        ## rosbag 2
        #red band
        # self.u= 500
        # self.v= 260


        ## rosbag3
        #far point
        self.u= 320
        self.v= 370
        #close point
        # self.u= 280
        # self.v= 140

        ## rosbag 5 #### to recalibrate
        #red band
        # self.u= 720
        # self.v= 270
        ################3 rosbags maqueta
        ## First
        # self.u= 720
        # self.v= 270

        ## Second
        # self.u= 870
        # self.v= 410

        ## Third ## nice to see effect of light
        # self.u= 870
        # self.v= 390

        ## Fourth
        # self.u= 730
        # self.v= 370

        # ## Fifth
        # self.u= 610
        # self.v= 300

        ## SingleBranch1
        # self.u= 610
        # self.v= 250

        ## SingleBranch2
        # self.u= 1050
        # self.v= 590

        ## SingleBranch3
        # self.u= 570
        # self.v= 330

    def reading_callback(self, color_image_rect, depth_image_rect):
        # rospy.loginfo("""Reconfigure Request: {left_crop_param}, {right_crop_param},{top_crop_param}, {bottom_crop_param}""".format(**config))
        ################################################# READING
        color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
        depth_image_rect=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)
        depth_image_rect_copy=np.copy(depth_image_rect)


        depth_image_rect_copy[depth_image_rect_copy >1500]=0
        depth_image_rect_copy[depth_image_rect_copy <350]=0

        image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2RGB)
        # things to do :
        # 0 get image crop around a radious in image
        # 1 search for cvresize way to see big  branch  in image crop
        # 2 use algorithm to take only branch of interest
        # 3 apply harris detector ( first study it )
        # we get the point from the hand and we crop around the interest point
        cv2.startWindowThread()
        self.get_point()
        image_point=np.copy(image)
        image_point = cv2.circle(image_point, (self.u,self.v), radius=3, color=(0, 0, 255), thickness=-1)
        image_point_area = cv2.circle(image_point, (self.u,self.v), radius=self.radius, color=(255, 0, 0), thickness=2)
        cv2.imshow('image',image_point)
        crop_img = image[self.v-self.radius:self.v+self.radius,self.u-self.radius:self.u+self.radius]
        crop_img_viewer = cv2.resize(crop_img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('crop_img',crop_img_viewer)

        # binarizing and segmenting
        r=242
        g=222
        b=130
        # crop_img[np.invert((crop_img[:,:,0]<b) & (crop_img[:,:,1]<g) & (crop_img[:,:,2]<r))]=0
        HLS_image = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HLS_FULL)
        lightness_thresh=100;
        crop_img[HLS_image[:,:,1]>lightness_thresh]=0        # depth_image_rect_copy[HLS_image[:,:,1]>lightness_thresh]=0

        gray_im = cv2.cvtColor(crop_img, cv2.COLOR_RGB2GRAY)
        ret,im_bin = cv2.threshold(gray_im,1,255,cv2.THRESH_BINARY)
        ret,im_bin_inv = cv2.threshold(gray_im,1,255,cv2.THRESH_BINARY_INV)

        crop_img_viewer = cv2.resize(crop_img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('crop_img_grey',crop_img_viewer)
        # get also crop from depth image
        crop_img_depth=depth_image_rect_copy[self.v-self.radius:self.v+self.radius,self.u-self.radius:self.u+self.radius]
        # cv_image_norm = cv2.normalize(crop_img_depth, None, 0, 255, cv2.NORM_MINMAX)
        # crop_img_viewer=cv2.resize(cv_image_norm, (500,500), interpolation = cv2.INTER_AREA)
        # cv2.imshow('crop_img_depth',crop_img_viewer.astype(np.uint8))

        crop_img_viewer_1=cv2.resize(im_bin_inv.astype(np.uint8), (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('im_bin_inv',crop_img_viewer_1)
        crop_img_viewer_1=cv2.resize(im_bin.astype(np.uint8), (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('im_bin',crop_img_viewer_1)

        crop_img_depth=scind.grey_dilation(crop_img_depth,(5,5))
        crop_img_depth[im_bin_inv.astype('bool')]=0
        cv_image_norm = cv2.normalize(crop_img_depth, None, 0, 255, cv2.NORM_MINMAX)
        crop_img_viewer_1=cv2.resize(cv_image_norm, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('crop_img_depth_masked',crop_img_viewer_1.astype(np.uint8))
        cv2.waitKey(0)

        ######################## now we have both a crop of depth and image
        # 2 use algorithm to take only branch of interest
        ####################### first try is watershed SEGMENTATION
        img=np.copy(crop_img)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,1,255,cv2.THRESH_BINARY)
        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 1)
        crop_img_viewer=cv2.resize(opening, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('noise removal',crop_img_viewer.astype(np.uint8))
        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=1)
        crop_img_viewer=cv2.resize(sure_bg, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow('sure background area',crop_img_viewer.astype(np.uint8))
        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        crop_img_viewer=cv2.resize(dist_transform, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("dist_transform",crop_img_viewer.astype(np.uint8))
        ret, sure_fg = cv2.threshold(dist_transform,0.2*dist_transform.max(),255,0)
        crop_img_viewer=cv2.resize(sure_fg, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("Finding sure foreground area",crop_img_viewer.astype(np.uint8))

        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg) # do i have it already ?
        crop_img_viewer=cv2.resize(unknown, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("Finding unknown region",crop_img_viewer.astype(np.uint8))
        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)
        crop_img_viewer=cv2.resize(markers.astype(np.uint8), (500,500), interpolation = cv2.INTER_AREA)
        crop_img_viewer = cv2.normalize(crop_img_viewer, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("connected components",crop_img_viewer.astype(np.uint8))

        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1
        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0
        np.set_printoptions(threshold=np.inf)
        # print('markers')
        # print(markers)
        # print(markers.shape)
        markers = cv2.watershed(img,markers)

        ## we need to find marker with max area.
        number_of_markers=markers.max()+1
        # print("inside for loop")
        # Iterate through markers and save result of area of each
        max_holder=[]
        for num in range(2, number_of_markers):
            area=np.count_nonzero(markers==num)
            max_holder.append((area,num))

        # chosing marker with max area
        max_holder=np.array(max_holder)
        # print(max_holder)
        # print(np.argmax(max_holder[:,0]))
        chosen_marker_idx=np.argmax(max_holder[:,0])
        chosen_marker=max_holder[chosen_marker_idx,1]
        branch_mask=((markers==chosen_marker).astype("bool")).astype(np.uint8)
        ## taking biggest branch outside of borders and background

        ret,branch_mask= cv2.threshold(branch_mask,0,255,cv2.THRESH_BINARY)
        crop_img_viewer=cv2.resize(branch_mask, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("branch_mask",crop_img_viewer.astype(np.uint8))
        img[markers == -1] = [255,0,0]
        crop_img_viewer=cv2.resize(img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("img",crop_img_viewer.astype(np.uint8))
        # cv2.imshow('im_bin',im_bin)



        # 3 apply harris detector ( first study it )
        ######## apply mask to image
        color_mask=np.copy(crop_img)
        color_mask[np.invert(branch_mask.astype("bool"))]=np.array([0,0,0])
        grey_mask= np.copy(gray_im)
        grey_mask[np.invert(branch_mask.astype("bool"))]=0
        crop_img_viewer=cv2.resize(color_mask, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("color_mask",crop_img_viewer.astype(np.uint8))
        crop_img_viewer=cv2.resize(grey_mask, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("grey_mask",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)

        # find Harris corners
        img = np.copy(color_mask)
        gray = np.float32(grey_mask)
        dst = cv2.cornerHarris(gray,2,3,0.04)
        # Threshold for an optimal value, it may vary depending on the image.
        img[dst>0.3*dst.max()]=[0,0,255]
        crop_img_viewer=cv2.resize(img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("img_harris_1",crop_img_viewer.astype(np.uint8))

        ########### trying with skel

        img = np.copy(branch_mask)
        dist_transform = cv2.distanceTransform(img,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.4*dist_transform.max(),255,0)
        crop_img_viewer=cv2.resize(sure_fg, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("sure_fg",crop_img_viewer.astype(np.uint8))

        dil=cv2.dilate(sure_fg,(3,3),iterations = 2)
        crop_img_viewer=cv2.resize(dil, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("sure_fg_dil",crop_img_viewer.astype(np.uint8))

        img=np.copy(dil.astype(np.uint8))
        size = np.size(img)
        skel = np.zeros(img.shape,np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
        done = False

        while( not done):
            eroded = cv2.erode(img,element)
            temp = cv2.dilate(eroded,element)
            temp = cv2.subtract(img,temp)
            skel = cv2.bitwise_or(skel,temp)
            img = eroded.copy()

            zeros = size - cv2.countNonZero(img)
            if zeros==size:
                done = True

        crop_img_viewer=cv2.resize(skel, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("skel",crop_img_viewer.astype(np.uint8))

        img = np.copy(color_mask)
        gray = np.float32(skel)
        dst = cv2.cornerHarris(gray,6,3,0.04)
        dst = cv2.dilate(dst,(3,3),iterations = 2)
        # Threshold for an optimal value, it may vary depending on the image.
        img[dst>0.6*dst.max()]=[0,0,255]
        crop_img_viewer=cv2.resize(img, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("img_harris_2",crop_img_viewer.astype(np.uint8))


        ##################3 new day, new skeletonization! with skwn
        # the idea is that the skel of branch will have nodes where the gems are!
        img = np.copy(color_mask)
        grey_mask=np.copy(grey_mask)
        th,bin_mask=cv2.threshold(grey_mask,0,255,cv2.THRESH_BINARY)
        # bin_mask[bin_mask==255]=1

        kernel= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_CLOSE, kernel)
        print(kernel)
        # bin_mask = cv2.erode(bin_mask,kernel,iterations = 2)
        crop_img_viewer=cv2.resize(bin_mask, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("bin_mask",crop_img_viewer.astype(np.uint8))
        #
        # ske = skeletonize(bin_mask).astype(np.uint16)
        # # build graph from skeleton
        # graph = sknw.build_sknw(ske)
        #
        # # draw image
        # plt.imshow(bin_mask, cmap='gray')
        #
        # # draw edges by pts
        # for (s,e) in graph.edges():
        #     ps = graph[s][e]['pts']
        #     plt.plot(ps[:,1], ps[:,0], 'green')
        #
        # # draw node by o
        # nodes = graph.nodes()
        # print(nodes[2])
        # ps = np.array([nodes[i]['o'] for i in nodes])
        # plt.plot(ps[:,1], ps[:,0], 'r.')
        #
        # # title and show
        # plt.title('Build Graph')
        # plt.show()
        # cv2.waitKey(0)
        #################################3 skeletonizaition using scipy
        skeleton = skeletonize(bin_mask).astype(np.uint8)
        skeleton=np.array(skeleton)
        print(skeleton)
        print(type(skeleton))
        skeleton_dil=cv2.dilate(skeleton.astype(np.uint8),(3,3),iterations = 2)
        print(skeleton_dil)
        crop_img_viewer=cv2.resize(skeleton_dil*255, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("skeleton_dil",crop_img_viewer.astype(np.uint8))
######################## using wand

        with ImgWand.from_array(bin_mask) as img:
            # Skeletonize
            img.morphology(method='thinning',
                           kernel='skeleton',
                           iterations=-1)
            wand_skel=np.copy(np.array(img))

        # Find line-junctions using Top-Hat Morphology
        # There are three kernels here, separated by a semi-colon
        # Each is rotated through 90 degress to form all 4 orientations
        # The first 3x3 kernel is the one tinted yellow in the diagram above
        # The second 3x3 kernel is the one tinted magenta in the diagram above
        # The third 3x3 kernel is the one tinted cyan in the diagram above
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
        with img.from_array(skeleton*255) as junctionsImage:
            junctionsImage.morphology(method='hit_and_miss', kernel=lineJunctions)
            junctions=np.copy(np.array(junctionsImage))


        crop_img_viewer=cv2.resize(junctions, (500,500), interpolation = cv2.INTER_AREA)
        cv2.imshow("junctions",crop_img_viewer.astype(np.uint8))
        junctions_dil = cv2.dilate(junctions,(3,3),iterations = 2)
        overlapped = cv2.addWeighted(junctions, 1, color_mask, 1, 0)
        crop_img_viewer=cv2.resize(overlapped, (700,700), interpolation = cv2.INTER_AREA)
        cv2.imshow("overlapped",crop_img_viewer.astype(np.uint8))
        cv2.waitKey(0)


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
