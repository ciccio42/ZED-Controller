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
        rospy.loginfo(f"Number of color frames {len(color_frames)}")
        rospy.loginfo(
            f"Color image dimension ({color_frames[0].width},{color_frames[0].height})")
        rospy.loginfo(f"Number of depth frames {len(depth_frames)}")
        rospy.loginfo(
            f"Color image dimension ({depth_frames[0].width},{depth_frames[0].height})")
        # if i == 0:
        #     for j in range(len(color_frames)):
        #         writers_rgb.append (cv2.VideoWriter(f'test_camera_{j+1}.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 30, (640,480)))
        #     i += 1

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
            rgb_frames.append(color_cv_image)
            depth_cv_image = bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
            print(depth_cv_image)
            if show_image:
                cv2.imshow("Color image", color_cv_image)
                cv2.imshow("Depth image", depth_cv_image)
                cv2.imwrite(f"{j}.png", color_cv_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        if len(rgb_frames) == 4:
            frames = []
            for i in range(2):
                row_frames = []
                for j in range(2):
                    index = i * 2 + j
                    if index < 4:
                        frame = rgb_frames[index]
                        row_frames.append(frame)
                row = cv2.hconcat(row_frames)
                frames.append(row)
            new_image = np.array(cv2.resize(
                cv2.vconcat(frames), (width, height)), np.uint8)
            cv2.imshow("Color image", new_image)
            # cv2.imshow("Depth image", depth_cv_image)
            cv2.waitKey(0)

        rate.sleep()

    # for j in len(writers_rgb):
    #     writers_rgb[i].release()
