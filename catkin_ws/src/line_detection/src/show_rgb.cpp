#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv/cv.h"

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

using namespace cv;

void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    std::cout << "Hey, listen!" << std::endl;

    try {
        cv_bridge::CvImagePtr cv_ptr;
            try
            {
              cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
              //cv_ptr = cv_bridge::toCvShare(msg);

            }
            catch (cv_bridge::Exception& e)
            {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return;
            }
        cv::Mat image;
        int padding = 10;
        image = cv_ptr->image(cv::Range(330-padding, 
                    450+padding), 
                cv::Range(0, cv_ptr->image.cols))
                .clone();
        //reset values
        int cut_topScanline = padding;
        int cut_bottomScanline = image.rows - padding;

        //make grey/white picture
        
        cvtColor(image, image, CV_RGB2GRAY);

        //Treshold to get binary picture
        threshold(image, image, 100, 500,THRESH_BINARY);

        //find edges
        Canny(image, image, 0, 2, 7, false);
        Size s = image.size();
        cv::circle(cv_ptr->image, cv::Point(50, 50), 100, CV_RGB(255,0,0));
        cv::imshow("foo",image);
        cv::waitKey(1);  // Update screen
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    std::cout << "Oh hai there!" << std::endl;

    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("camera/rgb/image_raw", MY_ROS_QUEUE_SIZE, imgcb);
    ros::Subscriber sub = nh.subscribe("camera/rgb/image_rect_color", MY_ROS_QUEUE_SIZE, imgcb);

    cv::namedWindow("foo");
    ros::spin();
    cv::destroyWindow("foo");

    std::cout << "byebye my friend" << std::endl;

    return 0;
}

