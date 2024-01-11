#! /bin/python3
import rospy
import cv2
from zed_camera_controller.srv import *
from cv_bridge import CvBridge
import numpy as np


import rospy
import pickle as pkl
import tf2_ros
import sys
import cv2
import argparse
import logging
import glob
import os
import debugpy
import math

PI = np.pi
EPS = np.finfo(float).eps * 4.0

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
logger = logging.getLogger("BB-Creator")

object_loc = []
cnt = 0
press = False

T_aruco_table = np.array([[-1.0, 0.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0, 0.06],
                          [0.0, 0.0, 1.0, 0.0],
                          [0, 0, 0, 1]])
T_aruco_bl = T_aruco_table  @ np.array([[-1, 0.0, 0, 0.01],
                                        [0.0, -1.0, 0, 0.612],
                                        [0, 0, 1, 0.120],
                                        [0, 0, 0, 1]])

camera_intrinsic = np.array([[345.2712097167969, 0.0, 337.5007629394531],
                             [0.0, 345.2712097167969,
                              179.0137176513672],
                             [0, 0, 1]])


film_px_offset = np.array([[337.5007629394531],
                           [179.0137176513672]])

ENV_OBJECTS = {
    'pick_place': {
        'obj_names': ['greenbox', 'yellowbox', 'bluebox', 'redbox', 'bin'],
        'bin_position': [0.18, 0.00, 0.75],
        'obj_dim': {'greenbox': [0.05, 0.055, 0.045],  # W, H, D
                    'yellowbox': [0.05, 0.055, 0.045],
                    'bluebox': [0.05, 0.055, 0.045],
                    'redbox': [0.05, 0.055, 0.045],
                    'bin': [0.6, 0.06, 0.15]},

        "id_to_obj": {0: "greenbox",
                      1: "yellowbox",
                      2: "bluebox",
                      3: "redbox"}
    },

    'nut_assembly': {
        'obj_names': ['nut0', 'nut1', 'nut2'],
        'ranges': [[0.10, 0.31], [-0.10, 0.10], [-0.31, -0.10]]
    },

    'camera_names': {'camera_front', 'camera_lateral_right', 'camera_lateral_left'},

    'camera_fovx': 345.27,
    'camera_fovy': 345.27,

    'camera_pos': {'camera_front': [-0.002826249197217832,
                                    0.45380661695322316,
                                    0.5322894621129393],
                   'camera_lateral_right': [-0.3582777207605626,
                                            -0.44377700364575223,
                                            0.561009214792732],
                   'camera_lateral_left': [-0.32693157973832665,
                                           0.4625646268626449,
                                           0.5675614538972504]},

    'camera_orientation': {'camera_front': [-0.00171609,
                                            0.93633855,
                                            -0.35105349,
                                            0.00535055],
                           'camera_lateral_right': [0.8623839571785069,
                                                    -0.3396500629838305,
                                                    0.12759260213488172,
                                                    -0.3530607214016715],
                           'camera_lateral_left': [-0.305029713753832,
                                                   0.884334094984367,
                                                   -0.33268049448458464,
                                                   0.11930536771213586]}
}


def mouse_drawing(event, x, y, flags, params):
    global object_loc, cnt, press
    if event == cv2.EVENT_LBUTTONDOWN:
        object_loc.append([x, y])
        cnt += 1
        press = True


def check_pick(step):
    logger.info("Checking picking condition")
    if step['action'][-1] != 0:
        return True
    else:
        return False


def check_init(step):
    if step['obs'].get("obj_bb", None) is None:
        return True
    else:
        return False


def plot_bb(img, obj_bb):

    # draw bb
    for obj_name in obj_bb.keys():
        center = obj_bb[obj_name]['center']
        upper_left_corner = obj_bb[obj_name]['upper_left_corner']
        bottom_right_corner = obj_bb[obj_name]['bottom_right_corner']
        img = cv2.circle(
            img, center, radius=1, color=(0, 0, 255), thickness=-1)
        img = cv2.rectangle(
            img, upper_left_corner,
            bottom_right_corner, (255, 0, 0), 1)
    cv2.imwrite("test_bb.png", img)
    cv2.imshow("Test", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def quat2mat(quaternion):
    """
    Converts given quaternion to matrix.

    Args:
        quaternion (np.array): (x,y,z,w) vec4 float angles

    Returns:
        np.array: 3x3 rotation matrix
    """
    # awkward semantics for use with numba
    inds = np.array([3, 0, 1, 2])
    q = np.asarray(quaternion).copy().astype(np.float32)[inds]

    n = np.dot(q, q)
    if n < EPS:
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q2 = np.outer(q, q)
    return np.array(
        [
            [1.0 - q2[2, 2] - q2[3, 3], q2[1, 2] - q2[3, 0], q2[1, 3] + q2[2, 0]],
            [q2[1, 2] + q2[3, 0], 1.0 - q2[1, 1] - q2[3, 3], q2[2, 3] - q2[1, 0]],
            [q2[1, 3] - q2[2, 0], q2[2, 3] + q2[1, 0], 1.0 - q2[1, 1] - q2[2, 2]],
        ]
    )


if __name__ == "__main__":
    i = 0
    show_image = True
    writers_rgb = []

    rospy.init_node('frames_tester', anonymous=True)

    rospy.wait_for_service('get_frames')
    get_frames_client = rospy.ServiceProxy('get_frames', GetFrames)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    bridge = CvBridge()

    rate = rospy.Rate(30)

    rospy.loginfo("Start getting frames")

    t = 0
    while not rospy.is_shutdown():

        exception = True
        while exception:
            try:
                # Get TCP Pose
                tcp_pose = tfBuffer.lookup_transform(
                    'base_link', 'tcp_link', rospy.Time())
                rospy.loginfo(f"TCP Pose {tcp_pose}")
                exception = False
            except ((tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)) as e:
                exception = True
                rospy.logerr(e)
        TCP_aruco = T_aruco_bl @ np.array([
            [tcp_pose.transform.translation.x],
            [tcp_pose.transform.translation.y],
            [tcp_pose.transform.translation.z],
            [1]])
        rospy.loginfo(f"TCP_aruco {TCP_aruco}")

        frames = get_frames_client()
        color_frames = frames.color_frames
        depth_frames = frames.depth_frames
        for indx, frame in enumerate(color_frames):
            if indx == 0:
                color_cv_image = bridge.imgmsg_to_cv2(
                    frame, desired_encoding='rgba8')
                # 1. Compute rotation_camera_to_world ()
                camera_quat = ENV_OBJECTS['camera_orientation']["camera_front"]
                r_aruco_camera = quat2mat(
                    np.array(camera_quat))
                p_aruco_camera = ENV_OBJECTS['camera_pos']["camera_front"]

                camera_quat = ENV_OBJECTS['camera_orientation']["camera_front"]
                r_camera_aruco = quat2mat(
                    np.array(camera_quat)).T
                p_camera_aruco = -np.matmul(r_camera_aruco, np.array(
                    [ENV_OBJECTS['camera_pos']["camera_front"]]).T)
                T_camera_aruco = np.append(
                    r_camera_aruco, p_camera_aruco, axis=1)

                # cv2.imshow(f'Frame {t}', color_cv_image)
                # cv2.setMouseCallback(f'Frame {t}', mouse_drawing)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                t += 1

                # convert central point to image
                rospy.loginfo(f"TCP_aruco {TCP_aruco}")
                rospy.loginfo(f"T_camera_aruco {T_camera_aruco}")
                tcp_camera = T_camera_aruco @ TCP_aruco
                tcp_camera = tcp_camera/tcp_camera[2][0]
                rospy.loginfo(f"TCP camera {tcp_camera}")
                tcp_pixel_cord = np.array(
                    camera_intrinsic @ tcp_camera, dtype=np.uint32)
                rospy.loginfo(f"Pixel coordinates {tcp_pixel_cord}")

                # plot point
                color_cv_image = cv2.circle(
                    color_cv_image, (tcp_pixel_cord[0][0], tcp_pixel_cord[1][0]), radius=1, color=(0, 0, 255), thickness=-1)
                cv2.imshow(f'Frame {t}', color_cv_image)
                cv2.waitKey(500)
                cv2.destroyAllWindows()
