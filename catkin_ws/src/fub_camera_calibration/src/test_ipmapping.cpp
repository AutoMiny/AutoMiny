#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iomanip>
#include <eigen3/Eigen/Dense>


#include "opencv2/opencv_modules.hpp"
#include "fub_camera_calibration/IPMapper.h"

#include <dynamic_reconfigure/server.h>
#include <fub_camera_calibration/calibrationConfig.h>
#include <fub_modelcar_tools/fub_modelcar_tools.h>

#include <iostream>
#include <string>
#include <fstream>    
using namespace std;
using namespace cv;

std::string TOPIC_NAME = "/camera/color/image_raw";
double f_u = 614.1699;
double f_v = 614.9002;
double c_u = 329.9491;
double c_v = 237.2788;
cv::Mat  intrinsicMatrix = (Mat1d(3, 3) << f_u, 0, c_u, 0, f_v, c_v, 0, 0, 1);

IPMapper ipMapper;
double cam_w=640;
double cam_h_half=480;
double pitch=12.8;
double height=18;
cv::Mat frame;
std::string file_name="/cfg/CameraParams.xml";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        Mat remapped_image = ipMapper.remap(frame);
        flip(remapped_image,remapped_image,0);
        cv::imshow("src image", frame);
        cv::imshow("IPmapped image", remapped_image);
        cv::waitKey(1);


    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}

void callback(fub_camera_calibration::calibrationConfig &config, uint32_t level) {

    height=config.height;
    pitch=config.pitch;
    ipMapper = IPMapper(cam_w, cam_h_half, f_u, f_v, c_u, c_v,pitch, height);
    if (frame.rows>0)
    {
        Mat remapped_image = ipMapper.remap(frame);
        flip(remapped_image,remapped_image,0);
        cv::imshow("src image", frame);
        cv::imshow("IPmapped image", remapped_image);
        cv::waitKey(1);
    }
}


void readFromFile()
{
    
    //CAMERA PARAMETERS:
    std::vector<fub_modelcar_tools::CameraCalibrationData> camera_params;

    fub_modelcar_tools::restoreXML(camera_params,file_name.c_str());

    f_u = camera_params.at(0).intrinsicMatrix.at<double>(0,0);//focal lense values (mm)
    f_v = camera_params.at(0).intrinsicMatrix.at<double>(1,1);
    
    c_u = camera_params.at(0).intrinsicMatrix.at<double>(0,2);//camera optical center
    c_v = camera_params.at(0).intrinsicMatrix.at<double>(1,2);

    pitch=camera_params.at(0).pitch;
    height=camera_params.at(0).height;

    std::cout << pitch << "\n";
    std::cout << height << "\n";
    // std::ifstream file(file_name.c_str());
    // // Read the entire file into memory
    // std::string s;
    // std::string t;
    // while (std::getline(file,t))
    //     s += t + '\n';
    // file.close();
    // // different member versions of find in the same order as above:
    // std::size_t index_pitch = s.find("pitch: ")+7;
    // std::string rest = s.substr(index_pitch);
    // index_pitch = rest.find("\n");
    // std::string str_pitch = rest.substr(0,index_pitch);
    // std::string str_height= rest.substr(index_pitch+9);
    // if (str_pitch!="")
    // {
    //     pitch= std::stod(str_pitch);
    //     std::cout << pitch << "\n";
    // }
    // if (str_height!="")
    // {
    //     double height= std::stod(str_height);
    //     std::cout << height << "\n";
    // }         
}
int main(int argc, char **argv) {

    ros::init(argc, argv, "ipMapper");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<fub_camera_calibration::calibrationConfig> server;
    dynamic_reconfigure::Server<fub_camera_calibration::calibrationConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    std::string path = ros::package::getPath("fub_camera_calibration");
    file_name =  path+"/cfg/CameraParams.xml";

    TOPIC_NAME = nh.param<std::string>("TOPIC_NAME", "/camera/color/image_raw");

    cv::startWindowThread();
    readFromFile();
    
    ipMapper = IPMapper(cam_w, cam_h_half, f_u, f_v, c_u, c_v,pitch, height);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1, imageCallback);
    ros::spin();
    ros::shutdown();

    return 0;
}
