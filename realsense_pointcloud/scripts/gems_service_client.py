#!/usr/bin/env python3

import rospy
import rospkg

from realsense_pointcloud.srv import gems_service, gems_serviceResponse # importing srv up and bottom
from geometry_msgs.msg import PoseStamped

def gems_service_client():
    rospy.wait_for_service('gems_service_srv') # wait for service to be available
    try:
        rospy.logwarn("here") # service is available
        client_handle = rospy.ServiceProxy('gems_service_srv', gems_service) # calling the service that we want to call
     	# wait topic
#rostopic pub /finger_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "camera_color_optical_frame"}, pose: {position: {x: 0.5, y: -0.15, z: 1.0}, orientation: {w: 1.0}}}'
        #rospy.logwarn(detections.poses)
        finger_pose_msg=PoseStamped()
        finger_pose_msg.header.frame_id="camera_color_optical_frame"
        finger_pose_msg.pose.position.x=0.5
        finger_pose_msg.pose.position.y=-0.15
        finger_pose_msg.pose.position.z=1
        finger_pose_msg.pose.orientation.w=1
        rospy.logwarn("here 1")
        response1 = client_handle(finger_pose_msg) # passing the two received messages to the service and saving the response
        rospy.logwarn("here 2")
        rospy.logwarn(response1)
        return response1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('gems_service_client') # initializing client
    gems_service_client()
