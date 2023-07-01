#! /bin/python3
import rospy
import cv2
from zed_camera_controller.srv import *
from cv_bridge import CvBridge
import numpy as np

if __name__ == "__main__":
    i = 0
    show_image = True
    writers_rgb = []

    rospy.init_node('frames_tester', anonymous=True)

    rospy.wait_for_service('get_frames')
    get_frames_client = rospy.ServiceProxy('get_frames', GetFrames)

    bridge = CvBridge()

    rate = rospy.Rate(30)

    rospy.loginfo("Start getting frames")

    while not rospy.is_shutdown():
        frames = get_frames_client()
        color_frames = frames.color_frames
        depth_frames = frames.depth_frames
        # rospy.loginfo(f"Number of color frames {len(color_frames)}")
        # rospy.loginfo(
        #     f"Color image dimension ({color_frames[0].width},{color_frames[0].height})")
        # # rospy.loginfo(f"Number of depth frames {len(depth_frames)}")
        # rospy.loginfo(
        #     f"Color image dimension ({depth_frames[0].width},{depth_frames[0].height})")
        if i == 0:
            for j in range(len(color_frames)):
                writers_rgb.append(cv2.VideoWriter(
                    f'test_camera_{j+1}.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (color_frames[0].width, color_frames[0].height)))
                print(writers_rgb)
            i += 1

        # show obtained frames

        rgb_frames = []
        width = None
        height = None
        for j, (color_msg, depth_msg) in enumerate(zip(color_frames, depth_frames)):
            color_cv_image = bridge.imgmsg_to_cv2(
                color_msg, desired_encoding='rgba8')
            if j == 0:
                height = color_cv_image.shape[0]
                width = color_cv_image.shape[1]
            color_cv_image = cv2.cvtColor(color_cv_image, cv2.COLOR_RGBA2RGB)
            rgb_frames.append(color_cv_image)

        for k in range(len(writers_rgb)):
            # rospy.loginfo("Write frame")
            # writers_rgb[k].write(rgb_frames[k])
            # depth_cv_image = bridge.imgmsg_to_cv2(
            #     depth_msg, desired_encoding='passthrough')
            # print(depth_cv_image)
            if show_image:
                cv2.imshow("Color image", color_cv_image)
                # cv2.imshow("Depth image", depth_cv_image)
                # cv2.imwrite(f"{j}.png", color_cv_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        rate.sleep()

    for j in len(writers_rgb):
        writers_rgb[j].release()
