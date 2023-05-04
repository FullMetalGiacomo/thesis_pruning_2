#!/usr/bin/env python3
import cv2
from matplotlib import pyplot as plt
import rospy
import time
# import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from std_msgs.msg import Header

import message_filters
from cv_bridge import CvBridge, CvBridgeError

from skspatial.objects import Plane, Points
from skspatial.plotting import plot_3d
from numpy.linalg import multi_dot

from sensor_msgs import point_cloud2

from realsense_pointcloud.srv import cables_service, cables_serviceResponse # importing srv up and bottom


class cable_preprocesser(object):
    def __init__(self):
        # Pvariables

        self.u=np.arange(0,1280,dtype=int)
        self.u=np.resize(self.u,(720,1280))
        self.v=np.array(np.arange(0,720))
        self.v=np.resize(self.v,(1280,720)).T

        # initializing cvbridge
        self.bridge = CvBridge()
        # Publishers
        self.pcl_cables_pub = rospy.Publisher('/cables_artificial_pointcloud', PointCloud2, queue_size=10)
        self.points_of_plane_pub = rospy.Publisher('/points_of_plane', PointCloud2, queue_size=10)
        self.points_of_plane_rot_pub = rospy.Publisher('/points_of_plane_rot', PointCloud2, queue_size=10)
        # var
        self.check=False
        # gems_serviceResponse
        self.cables_cloud=PointCloud2()

    def start(self):
        rospy.loginfo("Starting cable preprocesser")

        color_image_rect = message_filters.Subscriber('/camera_left_d435/color/image_raw', Image)
        depth_image_rect = message_filters.Subscriber('/camera_left_d435/aligned_depth_to_color/image_raw', Image)
        cam_inf=rospy.wait_for_message("/camera_left_d435/color/camera_info", CameraInfo)
        self.K=cam_inf.K
        ts = message_filters.TimeSynchronizer([color_image_rect, depth_image_rect], 10)

        ts.registerCallback(self.reading_callback)
        rospy.loginfo("cables_preprocess_is_working")
        return
        # rospy.spin()

    def publishPC2(self,cloud_array,depth_image_rect):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        header = depth_image_rect.header
        # header.frame_id = "camera_left_d435_link"
        # header.stamp = rospy.Time.now()
        #
        # x, y = np.meshgrid(np.linspace(-2,2,width), np.linspace(-2,2,height))
        # z = 0.5 * np.sin(2*x - count/10.0) * np.sin(2*y)
        points = np.array([cloud_array[:,0],cloud_array[:,1],cloud_array[:,2],cloud_array[:,2]]).reshape(4,-1).T

        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.cables_cloud=pc2
        self.pcl_cables_pub.publish(pc2)

    def publishPC2_points_of_plane(self,cloud_array,depth_image_rect):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        header = depth_image_rect.header
        points = np.array([cloud_array[:,0],cloud_array[:,1],cloud_array[:,2],cloud_array[:,2]]).reshape(4,-1).T

        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.points_of_plane_pub.publish(pc2)

    def publishPC2_points_of_plane_rotated(self,cloud_array,depth_image_rect):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        header = depth_image_rect.header
        points = np.array([cloud_array[:,0],cloud_array[:,1],cloud_array[:,2],cloud_array[:,2]]).reshape(4,-1).T

        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.points_of_plane_rot_pub.publish(pc2)

    def extract_pcl(self, ros_point_cloud): # gets pcl to xyz nparray (also rgb if needed)
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        gen = point_cloud2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)

        return xyz

    def reading_callback(self,color_image_rect, depth_image_rect):
        if self.check==False:
            ################################################# READING
            start_time = time.time()
            color_image_rect=np.frombuffer(color_image_rect.data, dtype=np.uint8).reshape(color_image_rect.height, color_image_rect.width, -1)
            depth_image_rect_copy=np.frombuffer(depth_image_rect.data, dtype=np.uint16).reshape(depth_image_rect.height, depth_image_rect.width)
            depth_image_rect_copy_cables=np.copy(depth_image_rect_copy)
            color_image_rect_copy=np.copy(color_image_rect)
            color_image_rect_copy = cv2.cvtColor(color_image_rect_copy, cv2.COLOR_BGR2RGB)
            # cv2.imshow('color_image_rect_copy',color_image_rect_copy)
            # cv2.waitKey(0)
            HLS_image = cv2.cvtColor(color_image_rect, cv2.COLOR_BGR2HLS_FULL)

            ################################################# CABLES SEGMENTATION https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
            lightness_thresh_cables=150
            depth_image_rect_copy_cables[HLS_image[:,:,1]>lightness_thresh_cables]=0

            ################ image processing to get best thinned image
            gray_im = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
            gray_im_plant = cv2.cvtColor(color_image_rect, cv2.COLOR_RGB2GRAY)
            # cv2.imshow('gray_im',gray_im)
            # cv2.waitKey(0)
            blur_im = cv2.GaussianBlur(gray_im, (3,3),1)
            blur_im_plant = cv2.GaussianBlur(gray_im_plant, (11,11),1)

            binarized_im = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,5,3)

            # cv2.imshow('thinned_bin_1_ghuoall',thinned1)
            thinned_im = cv2.ximgproc.thinning(binarized_im)

            ################# Apply Hough transform
            cdst = cv2.cvtColor(thinned_im, cv2.COLOR_GRAY2BGR)
            cdstP = np.copy(cdst)
            # Hough values tuned on italian rosbags
            hough_tresh=150
            minLineLength_value=150
            maxLineGap_value =10

            # try:
            linesP = cv2.HoughLinesP(thinned_im, rho=5,theta=np.pi/180,threshold=hough_tresh, lines=None,minLineLength=minLineLength_value,maxLineGap=maxLineGap_value)
            angle_list=[]
            b_list=[]
            if linesP is not None:
                for i in range(0, len(linesP)):

                    l = linesP[i][0]
                    angle = np.arctan((l[3]-l[1]) /(l[2]-l[0])) *180/np.pi
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
            # in here we store the slope and the +b coefficient
            lines_list = np.round(np.array([b_list,angle_list]),1)

            ################# at the end no classification was made.

            chosen_lines = lines_list[:,:]

            chosen_lines_im_thinned = cv2.cvtColor(thinned_im, cv2.COLOR_GRAY2BGR)
            only_cables_image = np.zeros((depth_image_rect.height,depth_image_rect.width,3), np.uint8)

            # draw lines on image!
            holder_vector=[]
            lines_borders_pixels=[]
            if chosen_lines is not None:
                for i in range(0, len(chosen_lines[0])):
                    r = 255
                    g = 0
                    b = 0
                    u1 = 0
                    v1 = int(chosen_lines[0,i])
                    u2 = 1280
                    v2 = int(u2*np.tan(chosen_lines[1,i]*np.pi/180)+chosen_lines[0,i])
                    holder_vector=np.array([u1, v1, u2, v2])
                    lines_borders_pixels.append(holder_vector)
                    cv2.line(chosen_lines_im_thinned, (u1, v1), (u2, v2), (b,g,r), 2, cv2.LINE_AA)
                    cv2.line(only_cables_image, (u1, v1), (u2, v2), (255,255,255), 2, cv2.LINE_AA)
                    cv2.line(color_image_rect_copy, (u1, v1), (u2, v2), (b,g,r), 2, cv2.LINE_AA)

            lines_borders_pixels=np.array(lines_borders_pixels) # u1 v1 u2 v2


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


            # cv2.imshow("Detected Lines enlarged - Probabilistic Line Transform", chosen_lines_im_thinned)
            # cv2.imshow("on color", color_image_rect_copy)
            # cv2.waitKey(0)
            # rospy.loginfo("histogram part")
            # plt.figure("histogram")
            # angle_list_sorted=np.sort(angle_list)
            # hist,bins = np.histogram(angle_list_sorted)
            # rospy.loginfo(hist)
            # rospy.loginfo(bins)
            # rospy.loginfo(angle_list_sorted)
            # plt.bars(hist, bins)

            # plt.show()

            ############### Creation of cables pointcloud

            only_cables_gray_image = cv2.cvtColor(only_cables_image, cv2.COLOR_RGB2GRAY)
            th, binarized_only_cables_im = cv2.threshold(only_cables_gray_image, 128, 255, cv2.THRESH_BINARY_INV)
            th, binarized_only_cables_im_plane = cv2.threshold(only_cables_gray_image, 128, 255, cv2.THRESH_BINARY)
            # cv2.imshow("binarized_only_cables_im", binarized_only_cables_im)
            # print(type(binarized_only_cables_im))
            binarized_only_cables_im.dtype='bool'


            depth_image_rect_copy_cables[binarized_only_cables_im]=0


            ##############3 i need to create a plane on which the lines are staying.
            ################# select only close distance points

            plane_points_generators=np.copy(depth_image_rect_copy_cables)
            plane_points_generators[plane_points_generators[:,:] < 400]=0
            plane_points_generators[plane_points_generators[:,:] > 1200]=0 # its millimiters!


            plane_thresh_im=np.copy(plane_points_generators)

            #fake depth image for plane creation

            fake_image = np.zeros(shape=(720,1280,3))
            fake_image[:,:,0]=self.u
            fake_image[:,:,1]=self.v
            fake_image[:,:,2]= plane_points_generators # taking the depth information of cables
            fake_image_vector=np.reshape(fake_image,(1280*720,3))
            fake_image_vector = fake_image_vector[(fake_image_vector[:,2] != 0)] #removing depth rows =0


            # randomly reducing vector size
            remove_idx= np.random.randint(0,fake_image_vector.shape[0],int(fake_image_vector.shape[0]*0.9)) # removing 0.n% of data
            fake_image_vector=np.delete(fake_image_vector, remove_idx, axis=0)

            #transforming to xyz coordinates "reconstruction vector pcl"
            reco_vector_pcl= np.zeros(shape=(fake_image_vector.shape[0],3))
            reco_vector_pcl[:,0]=(fake_image_vector[:,2]/self.K[0])*(fake_image_vector[:,0]-self.K[2])
            reco_vector_pcl[:,1]=(fake_image_vector[:,2]/self.K[4])*(fake_image_vector[:,1]-self.K[5])
            reco_vector_pcl[:,2]=fake_image_vector[:,2]

            # creating plane
            # self.publishPC2_points_of_plane(reco_vector_pcl/1000,depth_image_rect) # visualization purposes
            points = Points(reco_vector_pcl)
            plane = Plane.best_fit(points)


            ######################33 if the points are too dispersed wrt to plane we don't want to find the lines

             ## moving points to center and find the reference frame of plane ( z = normal plane)
            test_points=np.copy(reco_vector_pcl)/1000
            test_points_plane=np.copy(reco_vector_pcl)/1000
            # rospy.loginfo("mean of test points")
            # rospy.loginfo(np.mean(test_points[:,2]))
            test_points_plane[:,0]=test_points_plane[:,0]-plane.point[0]/1000
            test_points_plane[:,1]=test_points_plane[:,1]-plane.point[1]/1000
            test_points_plane[:,2]=test_points_plane[:,2]-plane.point[2]/1000

            test_points[:,0]=test_points[:,0]-np.mean(test_points[:,0])
            test_points[:,1]=test_points[:,1]-np.mean(test_points[:,1])
            test_points[:,2]=test_points[:,2]-np.mean(test_points[:,2])
            u1=np.cross(plane.normal,[0,0,1]) # first vector orthogonal to normal
            u1=u1/np.linalg.norm(u1) # normalizing
            u2=np.cross(plane.normal,u1)# second vector orthogonal to normal
            R=np.array([(u1),(u2),(plane.normal)]).T # rotational matrix of the plane
            rotated_test_points=np.dot(test_points_plane,R)

            x_std_transformed=np.var(rotated_test_points[:,0])
            y_std_transformed=np.var(rotated_test_points[:,1])
            z_std_transformed=np.var(rotated_test_points[:,2])
            magnitude_std_transformed=np.linalg.norm([x_std_transformed,y_std_transformed,z_std_transformed])

            x_std=np.std(test_points[:,0])
            y_std=np.std(test_points[:,1])
            z_std=np.std(test_points[:,2])
            magnitude_std=np.linalg.norm([x_std,y_std,z_std])


            if z_std_transformed>0.03:
                rospy.logerr("Could not create plane, points too dispersed")
                return
            else:
                ####################################3 trying to clean from plane rotation/projection points!
                ### idea is to clean the lower part of rotated points.
                clear_idx= ((rotated_test_points[:,2]>np.percentile(rotated_test_points[:,2],20)) & (rotated_test_points[:,2]<np.percentile(rotated_test_points[:,2],80)))
                rotated_test_points=rotated_test_points[clear_idx,:]
                ## derotate
                derotated_test_points= np.dot(rotated_test_points,np.linalg.inv(R))
                # retranslate them and publish
                derotated_test_points[:,0]= derotated_test_points[:,0] + plane.point[0]/1000
                derotated_test_points[:,1]= derotated_test_points[:,1] + plane.point[1]/1000
                derotated_test_points[:,2]= derotated_test_points[:,2] + plane.point[2]/1000

                # self.publishPC2_points_of_plane_rotated(derotated_test_points,depth_image_rect)
                ## rebuild plane
                points = Points(derotated_test_points*1000)
                plane = Plane.best_fit(points)

                ################################## Now we have the equation of the plane where lines are
                ### Trying out an algorithm.
                xy_coord_list=[]
                lines_borders_pixels=np.reshape(lines_borders_pixels,(lines_borders_pixels.shape[0]*2,2))
                x0 = plane.point[0]
                y0 = plane.point[1]
                z0 = plane.point[2]
                a = plane.normal[0]
                b = plane.normal[1]
                c = plane.normal[2]

                for i in range(len(lines_borders_pixels)):

                    u1=lines_borders_pixels[i,0]
                    v1=lines_borders_pixels[i,1]

                    A11=-a*(u1-self.K[2])/(self.K[0]*c)
                    A12=-b*(u1-self.K[2])/(self.K[0]*c)
                    A21=-a*(v1-self.K[5])/(self.K[4]*c)
                    A22=-b*(v1-self.K[5])/(self.K[4]*c)
                    A=np.matrix(np.array([[A11,A12],[A21,A22]]))
                    I=np.matrix(np.identity(2))

                    B11=-A11
                    B12=-A12
                    B13=(u1-self.K[2])/self.K[0]
                    B21=-A21
                    B22=-A22
                    B23=(v1-self.K[5])/self.K[4]
                    B=np.matrix(np.array([[B11,B12,B13],[B21,B22,B23]]))
                    P=np.matrix(np.array([[x0],[y0],[z0]]))
                    A_I_inv=np.matrix(I.A-A.A).I

                    x_vector=multi_dot([A_I_inv.A, B.A, P.A])
                    xy_coord_list.append(x_vector.T)

                xy_coord_list=np.array(xy_coord_list) # contains xy
                xy_coord_list=np.reshape(xy_coord_list,(xy_coord_list.shape[0],2))


                xyz_coord_list=np.zeros(shape=(xy_coord_list.shape[0],3)) # contains xyz (n,xyz)
                xyz_coord_list[:,0]=xy_coord_list[:,0]
                xyz_coord_list[:,1]=xy_coord_list[:,1]
                xyz_coord_list[:,2]=z0+(x0-xy_coord_list[:,0])*a/c+(y0-xy_coord_list[:,1])*b/c

                xyz_coord_list=np.reshape(xyz_coord_list,(int(xyz_coord_list.shape[0]*0.5),6)) #contains x1y1z1 x2y2z2 (xyz,xyz)

                ###################################3 find 3d lines
                # direction_ratios!
                lmn=np.zeros(shape=((xyz_coord_list.shape[0],3)))
                lmn[:,0]=(xyz_coord_list[:,3]-xyz_coord_list[:,0])
                lmn[:,1]=(xyz_coord_list[:,4]-xyz_coord_list[:,1])
                lmn[:,2]=(xyz_coord_list[:,5]-xyz_coord_list[:,2])

                number_of_points_per_line=1000
                x=np.linspace(-700,700,number_of_points_per_line)
                cloud_array=[]
                holder_xyz_lines=np.zeros(shape=(number_of_points_per_line,3))
                holder_xyz_lines[:,0]=x
                for j in range (len(lmn)): # building lines
                    y=(lmn[j,1]*(x-xyz_coord_list[j,0]))/lmn[j,0]+xyz_coord_list[j,1]
                    z=(lmn[j,2]*(x-xyz_coord_list[j,0]))/lmn[j,0]+xyz_coord_list[j,2]
                    holder_xyz_lines[:,1]=y
                    holder_xyz_lines[:,2]=z
                    cloud_array.append(np.array(holder_xyz_lines))

                cloud_array=np.array(cloud_array)/1000 # from mm to meters


                cloud_array=np.reshape(cloud_array,(int(cloud_array.shape[0]*number_of_points_per_line),3))
                cloud_array=cloud_array[(cloud_array[:,1]>-0.7)&(cloud_array[:,1]<0.7)] # cleaning upper and lower points
                cloud_array=cloud_array[(cloud_array[:,2]>0)&(cloud_array[:,1]<3)]

                ###################  tranforming to PointCloud2 msg
                self.publishPC2(cloud_array,depth_image_rect) # publish the cables as a pointcloud
                self.check=True
                print("Cables found")





def cables_req(req): # here we call the handler
    # grapes_wrt_tag_t = PoseArray()
    rospy.logwarn("Service request recieved!")
    try:
        class_cable = cable_preprocesser()
        class_cable.start()
        while class_cable.check==False:
            rospy.sleep(0.01)

        class_cable.check=False
        return cables_serviceResponse(cables_pcl=class_cable.cables_cloud)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


def cables_service_server():
    rospy.init_node('cables_service_node', anonymous=True) # initialize the service node
    rospy.loginfo("cables service node ready")
    s = rospy.Service('cables_service_srv',  cables_service, cables_req)
    rospy.spin()

if __name__ == '__main__':
    # detect_pub = rospy.Publisher('/PoseStampedArray',  PoseStampedArray, queue_size=10)
    cables_service_server()
