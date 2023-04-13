#include <string>
#include <exception>
// ZED includes
#include <sl/Camera.hpp>
#include "zed_camera_controller/GetFrames.h"
#include <opencv2/opencv.hpp>

#ifndef ZEDCAMERACONTROLLER_H
#define ZEDCAMERACONTROLLER_H

namespace zed_camera_controller{
    
    std::map<std::string, sl::RESOLUTION> RESOLUTION = {
        {"HD2K", sl::RESOLUTION::HD2K},
        {"HD1080", sl::RESOLUTION::HD1080},
        {"HD720", sl::RESOLUTION::HD720},
        {"VGA", sl::RESOLUTION::VGA},
    }; 

    std::map<std::string, sl::DEPTH_MODE> DEPTH_MODE = {
        {"NONE", sl::DEPTH_MODE::NONE},
        {"PERFORMANCE", sl::DEPTH_MODE::PERFORMANCE},
        {"QUALITY", sl::DEPTH_MODE::QUALITY},
        {"ULTRA", sl::DEPTH_MODE::ULTRA},
        {"NEURAL", sl::DEPTH_MODE::NEURAL}
    };

    std::map<std::string, sl::UNIT> UNIT = {
        {"MILLIMETER", sl::UNIT::MILLIMETER},
        {"CENTIMETER", sl::UNIT::CENTIMETER},
        {"METER", sl::UNIT::METER},
        {"INCH", sl::UNIT::INCH},
        {"FOOT", sl::UNIT::FOOT}
    };
    
    class ZEDCameraController{

        public:
            ZEDCameraController(std::vector<unsigned int> serials, sl::RESOLUTION resolution, int fps, sl::DEPTH_MODE depth_mode, sl::UNIT coordinate_units, float depth_minimum_distance, float depth_maximum_distance);
            ~ZEDCameraController();
            bool get_frame(zed_camera_controller::GetFrames::Request &req, zed_camera_controller::GetFrames::Response &res);
            cv::Mat slMat2cvMat(sl::Mat& input);
            int getOCVtype(sl::MAT_TYPE type);
            int get_width();
            int get_height();
            std::vector<double> get_intrinsic_matrix();
            std::vector<double> get_distorsion_parameters();
        private:
            std::vector<sl::Camera*> _zed_cameras;
            sl::InitParameters _init_camera_parameters;
            std::string _camera_name;
            sl::RuntimeParameters _runtime_parameters;
            int _img_width;
            int _img_height;
    };

}
#endif //ZEDCAMERACONTROLLER_H