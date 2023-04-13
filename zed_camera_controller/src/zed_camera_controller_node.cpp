#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>    
#include "zed_camera_controller.hpp"
#include "zed_camera_controller/GetFrames.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "env_camera_handler_node");
    ros::NodeHandle nh;

    // Get camera parameters from parameter server
    
    // Camera's id
    std::vector<unsigned int> serials;
    std::string camera_front_id;
    nh.getParam("/camera_id/camera_front_id", camera_front_id);
    serials.emplace_back((unsigned int) std::stoi(camera_front_id));
    ROS_INFO_STREAM("Camera front id: "<< camera_front_id);

    std::string camera_lateral_left_id;
    nh.getParam("/camera_id/camera_lateral_left_id", camera_lateral_left_id);
    serials.emplace_back((unsigned int) std::stoi(camera_lateral_left_id));
    ROS_INFO_STREAM("Camera lateral left: "<< camera_lateral_left_id);


    std::string camera_lateral_right_id;
    nh.getParam("/camera_id/camera_lateral_right_id", camera_lateral_right_id);
    serials.emplace_back((unsigned int) std::stoi(camera_lateral_right_id));
    ROS_INFO_STREAM("Camera lateral right: "<< camera_lateral_right_id);

    std::string camera_robot_id;
    nh.getParam("/camera_id/camera_robot_id", camera_robot_id);
    serials.emplace_back((unsigned int) std::stoi(camera_robot_id));
    ROS_INFO_STREAM("Camera robot id: "<< camera_robot_id);


    // Get camera parameters
    // 1. Resolution
    std::string resolution;
    nh.getParam("/camera_resolution", resolution);
    // 2. FPS
    int fps;
    nh.getParam("/fps", fps);
    // 3. Depth mode
    std::string depth_mode;
    nh.getParam("/depth_mode", depth_mode);
    // 4. Units
    std::string units;
    nh.getParam("/coordinate_units", units);
    // 5. Minumim distance and maximum distance
    float min_distance;
    float max_distance;
    nh.getParam("/depth_minumum_distance", min_distance);
    nh.getParam("/depth_maximum_distance", max_distance);
    
    ROS_INFO_STREAM("---- INIT MULTI-CAMERA CONTROLLER ----");
    zed_camera_controller::ZEDCameraController camera_controller(serials, zed_camera_controller::RESOLUTION[resolution], fps, zed_camera_controller::DEPTH_MODE[depth_mode], zed_camera_controller::UNIT[units], min_distance, max_distance);

    // Advertising service
    ros::ServiceServer get_frames_service = nh.advertiseService("get_frames", &zed_camera_controller::ZEDCameraController::get_frame, &camera_controller);
    ROS_INFO_STREAM("---- RUNNING SERVICE ----");

    // Publishing camera information on parameter server
    nh.setParam("img_width", camera_controller.get_width());
    nh.setParam("img_height", camera_controller.get_height());
    nh.setParam("intrinsic_matrix", camera_controller.get_intrinsic_matrix());
    nh.setParam("factor_depth", 1);
    nh.setParam("distorsion_parameters", camera_controller.get_distorsion_parameters());
    ros::spin();

}