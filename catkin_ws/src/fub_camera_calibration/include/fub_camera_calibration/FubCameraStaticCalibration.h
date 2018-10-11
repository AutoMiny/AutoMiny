#ifndef FUB_STATIC_CAMERA_CALIBRATION_H
#define FUB_STATIC_CAMERA_CALIBRATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp> 
#include <sensor_msgs/CameraInfo.h>
#include <fub_modelcar_tools/fub_modelcar_tools.h>
#include <fub_modelcar_tools/Object3DPoints.h>



using namespace std;
using namespace cv;

class FubCameraStaticCalibration 
{
public:
    FubCameraStaticCalibration(ros::NodeHandle nh);
    ~FubCameraStaticCalibration() {};


protected:

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    image_transport::ImageTransport it;


    ///subcribers
    image_transport::Subscriber sub_image;
    image_transport::Subscriber sub_camera_info;

    image_transport::Publisher image_pub_bi_gray;
    image_transport::Publisher image_pub_gray;


    /**
     * Intrinsic camera matrix for the raw (distorted) images.
           [fx  0 cx]
     * K = [ 0 fy cy]
           [ 0  0  1]
     * Projects 3D points in the camera coordinate frame to 2D pixel
      coordinates using the focal lengths (fx, fy) and principal point (cx, cy).*/
    fub_modelcar_tools::CameraCalibrationData camera_params;

    std::string image_topic_name;
    std::string file_name;
    std::string file_3d_points;
    std::string path;

    int y_seg;
    int x_seg;
    int offset_x;
    int offset_y;

    cv_bridge::CvImagePtr cv_ptr;
    std::vector<cv::Point3f> objectPoints;

    int bi_gray_max;
    int bi_gray_min;

    bool debug;


    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

        ROS_ERROR("1");
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);//TYPE_8UC1
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        static_calibration();
        writeToFile();
    }

    // void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
    // {
    //     //intrinsicMatrix=msg.K;
    //     //distCoeffs=msg.D;
    // }


    void gray (cv::Mat img);
    cv::Mat bi_gray (cv::Mat img);
    void writeToFile();
    std::vector<cv::Point3f> Generate3DPoints();
    std::vector<cv::Point2f> Generate2DPoints(cv::Mat img);
    void static_calibration();
    double calculateEuler(cv::Mat rmat);

};

#endif // FUB_STATIC_CAMERA_CALIBRATION_H