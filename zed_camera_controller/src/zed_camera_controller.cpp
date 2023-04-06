#include "zed_camera_controller.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

using namespace zed_camera_controller;

/**
 * @brief Construct a new ZedCameraController object
 * 
 * @param 
 */
ZEDCameraController::ZEDCameraController(std::vector<unsigned int> serials, sl::RESOLUTION resolution, int fps, sl::DEPTH_MODE depth_mode, sl::UNIT coordinate_units, float depth_minimum_distance, float depth_maximum_distance)
{

    // set init parameters
    for (auto &serial : serials)
    {
        this->_init_camera_parameters.input.setFromSerialNumber(serial);
        this->_init_camera_parameters.camera_resolution = resolution; 
        this->_init_camera_parameters.camera_fps = fps;
        this->_init_camera_parameters.depth_mode = depth_mode;
        this->_init_camera_parameters.coordinate_units = coordinate_units;
        this->_init_camera_parameters.depth_minimum_distance = depth_minimum_distance;
        this->_init_camera_parameters.depth_maximum_distance =  depth_maximum_distance; 

        // open camera stream
        if(serial != -1){
            sl::Camera* camera = new sl::Camera();
            sl::ERROR_CODE err = camera->open(this->_init_camera_parameters);
            if (err != sl::ERROR_CODE::SUCCESS){
                std::string error_msg = "Fail in open the camera";
                ROS_ERROR_STREAM("Fail in open the camera\nError code "<<err);
                throw std::runtime_error(error_msg);
            }
            this->_zed_cameras.push_back(camera);
            ROS_INFO_STREAM("Camera initialized " << camera->getCameraInformation().serial_number);
        }
        
    }

    // Set runtime parameters after opening the camera    
    this->_runtime_parameters.enable_fill_mode = true; // Use STANDARD sensing mode
    this->_img_width = this->_zed_cameras[0]->getCameraInformation().camera_configuration.resolution.width;
    this->_img_height = this->_zed_cameras[0]->getCameraInformation().camera_configuration.resolution.height;
}

ZEDCameraController::~ZEDCameraController()
{
    for (auto &camera : this->_zed_cameras){
        camera->~Camera();
    }
}

bool ZEDCameraController::get_frame(zed_camera_controller::GetFrames::Request &req, zed_camera_controller::GetFrames::Response &res){

    // Define image and depth matrices
    sl::Mat rgb_image(this->_img_width, this->_img_height, sl::MAT_TYPE::U8_C4);
    sl::Mat depth_image(this->_img_width, this->_img_height, sl::MAT_TYPE::F32_C1); 
    sl::Mat depth_image_normalized(this->_img_width, this->_img_height, sl::MAT_TYPE::U8_C4); 
    
    cv::Mat rgb_image_cv = ZEDCameraController::slMat2cvMat(rgb_image);
    cv::Mat depth_image_cv = ZEDCameraController::slMat2cvMat(depth_image);
    cv::Mat depth_image_cv_normalized = ZEDCameraController::slMat2cvMat(depth_image_normalized);

    for (auto camera : this->_zed_cameras)
    {
        if (camera->grab(this->_runtime_parameters) == sl::ERROR_CODE::SUCCESS){
            // 1. Get Color frame
            camera->retrieveImage(rgb_image, sl::VIEW::LEFT);

            // 2. Get Depth frame
            camera->retrieveMeasure(depth_image,sl::MEASURE::DEPTH);

            // 3. Convert Color image to sensor_msgs
            sensor_msgs::ImagePtr rgb_image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgba8", rgb_image_cv).toImageMsg();
            res.color_frames.push_back(*rgb_image_msg);
            
            // 4. Convert Depth image to sensor_msgs
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1",
            depth_image_cv).toImageMsg();
            res.depth_frames.push_back(*depth_msg);
            
        }else{
            // ToDo return error in service message
            return false;
        }
    }

    return true;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat ZEDCameraController::slMat2cvMat(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), ZEDCameraController::getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

// Mapping between MAT_TYPE and CV_TYPE
int ZEDCameraController::getOCVtype(sl::MAT_TYPE type){
    int cv_type = -1;
    switch (type) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
};
