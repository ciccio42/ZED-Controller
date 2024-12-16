#! /bin/python3
import rospy
import cv2
from zed_camera_controller.srv import *
from cv_bridge import CvBridge
import numpy as np
from aruco_detector.srv import ArucoDetection, ArucoDetectionResponse

Target_Position =  [0.022843813138628592, -0.43800020977692405, 0.5643843146648674]
Target_Orientation = [0.3603325062389276, 0.015749675284185274, -0.0008269422755895826, 0.9326905965230317]

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

        frames = get_frames_client()
        color_frame = frames.color_frames[0]

        # convert message to cv2 image
        color_cv_image = bridge.imgmsg_to_cv2(
            color_frame, desired_encoding='rgba8')
        cv2.imshow("Color", color_cv_image)
        cv2.waitKey(100)
        # cv2.destroyAllWindows()
        
        # 
        position = aruco_pose.camera_pose.position
        orientation = aruco_pose.camera_pose.orientation
        
        # compute difference
        position_diff = np.array([position.x, position.y, position.z]) - np.array(Target_Position)
        orientation_diff = np.array([orientation.x, orientation.y, orientation.z, orientation.w]) - np.array(Target_Orientation)
        if (position_diff < 0.001).all() and (orientation_diff < 0.001).all():
            rospy.loginfo("Calibration done")
            