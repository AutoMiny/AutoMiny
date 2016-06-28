#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;


void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    // The message's data is a raw buffer. While the type is uint8_t*, the
    // data itself is an array of uint16_t's (for IR data), the value of
    // the uint being BLA.
    std::cout << "Top-left corner: " << *reinterpret_cast<const uint16_t*>(&msg->data[0]) << std::endl;

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
        cv_ptr = cv_bridge::toCvShare(msg);

   
        // imshow expects a float value to lie in [0,1], so we need to normalize
        // for visualization purposes.
        double max = 0.0, min = 0.0;
        cv::minMaxLoc(cv_ptr->image, &min, &max, 0, 0);

        size_t w = cv_ptr->image.cols;
        size_t h = cv_ptr->image.rows;
        cv::Mat normalized(h, w, CV_32F/* cv_ptr->image.type */);
        for(size_t y = 0 ; y < h ; ++y) {
            for(size_t x = 0 ; x < w ; ++x) {
                normalized.at<float>(y,x) = cv_ptr->image.at<uint16_t>(y,x)/max;
            }
        }

        cv::imshow("foo", normalized);
        cv::waitKey(1);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "foo");

    std::cout << "Oh hai there!" << std::endl;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/ir/image_raw", MY_ROS_QUEUE_SIZE, imgcb);

    cv::namedWindow("foo");
    ros::spin();
    cv::destroyWindow("foo");

    std::cout << "byebye my friend" << std::endl;

    return 0;
}

