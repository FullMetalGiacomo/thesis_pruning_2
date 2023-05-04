#!/usr/bin/env python3

import rospy
import rospkg

from realsense_pointcloud.srv import cables_service, cables_serviceResponse # importing srv up and bottom
from sensor_msgs.msg import PointCloud2

def cables_service_client():
    rospy.wait_for_service('cables_service_srv') # wait for service to be available
    try:
        rospy.logwarn("here") # service is available
        client_handle = rospy.ServiceProxy('cables_service_srv', cables_service) # calling the service that we want to call
     	# wait topic
        rospy.logwarn("here 1")
        response1 = client_handle() 
        rospy.logwarn("here 2")
        rospy.logwarn(response1)
        return response1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('cables_service_client') # initializing client
    cables_service_client()
