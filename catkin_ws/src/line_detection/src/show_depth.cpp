#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    // The message's data is a raw buffer. While the type is uint8_t*, the
    // data itself is an array of floats (for depth data), the value of
    // the float being the distance in meters.
    std::cout << "Top-left corner: " << *reinterpret_cast<const float*>(&msg->data[0]) << "m" << std::endl;

    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        // imshow expects a float value to lie in [0,1], so we need to normalize
        // for visualization purposes.
        double max = 0.0;
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
        cv::Mat normalized;
        cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0)  ;

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
    ros::Subscriber sub = nh.subscribe("camera/depth/image", MY_ROS_QUEUE_SIZE, imgcb);

    cv::namedWindow("foo");
    ros::spin();
    cv::destroyWindow("foo");

    std::cout << "byebye my friend" << std::endl;

    return 0;
}

