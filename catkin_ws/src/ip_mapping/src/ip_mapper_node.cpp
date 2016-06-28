//
//  IPMapper.cpp
//  TestExtendedDetector
//
/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM . All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 **/

/***************************************************************************
 * $Author:: Zahra Boroujeni $  $Date:: 2015-11-05  $*
 ***************************************************************************/
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
#include "IPMapper.h"
#include <image_transport/image_transport.h>
#define PROJECTED_IMAGE_HEIGTH 250
//#define PAINT_OUTPUT

class online_IPM
{
  private:
    // the node handle
    ros::NodeHandle nh_;
    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    // subscribers
    ros::Subscriber read_images_;

    // publishers
    image_transport::ImageTransport it;
    image_transport::Publisher pub_mapped_images_;

    IPMapper ipMapper;

    
    

  public:

    // 
 
    // constructor
    online_IPM(ros::NodeHandle nh, int ow, int oh, double f_u, double f_v, double c_u, double c_v, double deg, double cam_h, std::string camera_name)
        : nh_(nh), priv_nh_("~"),ipMapper(ow,oh,f_u,f_v,c_u,c_v,deg,cam_h),it(nh_)
    {
        read_images_= nh_.subscribe(nh_.resolveName(camera_name), 1,&online_IPM::publish_remapper,this);
        pub_mapped_images_= it.advertise("/camera/ground_image_ipmapped", 1);
        //ipMapper.initialize(200,PROJECTED_IMAGE_HEIGTH, fu, fv, cx, cy, pitch);
    }

    // callback functions
    void publish_remapper(const sensor_msgs::Image::ConstPtr& msg);


    //! Empty stub
    ~online_IPM() {}


};


void online_IPM::publish_remapper(const sensor_msgs::Image::ConstPtr& msg)
{
  
    ROS_INFO("Publish remapper: true");
    
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);    
        Mat remapped_image;
        remapped_image=ipMapper.remap(cv_ptr->image);

         #ifdef PAINT_OUTPUT //3ms
             cv::imshow("IPMapped image", remapped_image);
             cv::waitKey(1);
         #endif
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::MONO8, remapped_image).toImageMsg();
        pub_mapped_images_.publish(msg);
    } 
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_IPM");
    ros::NodeHandle nh;

    std::string camera_name = "/usb_cam/image_raw";
    int ow = 640;                             //output width
    int oh = 480;                             //output height
    double f_u = 100;                         //focal lense values (mm)
    double f_v = 100;
    double c_u = 50;                          //camera optical center
    double c_v = 50;
    double deg = 45;                          //degree of camera
    double cam_h = 10;                        //height of camera

    nh.param<std::string>("camera_name", camera_name, "/usb_cam/image_raw"); 
    nh.param<int>("cam_w", ow, 640); 
    nh.param<int>("cam_h", oh, 480); 
    nh.param<double>("f_u", f_u, 624.650635); 
    nh.param<double>("f_v", f_v, 626.987244); 
    nh.param<double>("c_u", c_u, 309.703230); 
    nh.param<double>("c_v", c_v, 231.473613); 
    nh.param<double>("deg", deg, 27); 
    nh.param<double>("cam_height", cam_h, 18); 

    online_IPM node(nh,ow,oh,f_u,f_v,c_u,c_v,deg,cam_h,camera_name);

    while(ros::ok())
    {
        ros::spinOnce();

    }
    return 0;
}
