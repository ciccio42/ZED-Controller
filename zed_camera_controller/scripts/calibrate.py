#! /bin/python3
import rospy
import cv2
from zed_camera_controller.srv import *
from cv_bridge import CvBridge
import numpy as np
from aruco_detector.srv import ArucoDetection, ArucoDetectionResponse

if __name__ == "__main__":
    i = 0
    show_image = True
    writers_rgb = []

    rospy.init_node('calibrator', anonymous=True)

    rospy.wait_for_service('get_frames')
    get_frames_client = rospy.ServiceProxy('get_frames', GetFrames)

    rospy.wait_for_service('/aruco_detection')
    get_aruco_pose = rospy.ServiceProxy('/aruco_detection', ArucoDetection)

    bridge = CvBridge()

    rate = rospy.Rate(0.5)

    rospy.loginfo("Start getting frames")

    while not rospy.is_shutdown():
        rospy.loginfo("Asking for aruco pose")
        aruco_pose = get_aruco_pose()
        rospy.loginfo(f"Camera pose wrt table_1 frame \n {aruco_pose}")
        rate.sleep()
