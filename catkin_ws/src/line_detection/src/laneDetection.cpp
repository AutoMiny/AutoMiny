#include "laneDetection.h"


using namespace std;
using namespace cv;

//#define PAINT_OUTPUT

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

image_transport::CameraPublisher realsense_rgb_image_pub;

//msgs head
unsigned int head_sequence_id = 0;
ros::Time head_time_stamp;
std::string rgb_frame_id = "_rgb_optical_frame";
sensor_msgs::CameraInfoPtr rgb_camera_info;

cLaneDetection::cLaneDetection(ros::NodeHandle nh, int cam_w_, int cam_h_, int proj_y_start_,
        int proj_image_h_, int proj_image_w_, int roi_top_w_, int roi_bottom_w_, int proj_image_horizontal_offset_,
        int detector_size, int lane_width,
        std::string path_2features, std::string path_30features)
    : nh_(nh), priv_nh_("~"),detector(detector_size,Point(0,0),Point(proj_image_w_,proj_image_h_),
        proj_image_h_,proj_image_w_,roi_top_w_,roi_bottom_w_,path_2features, path_30features),model(true, proj_image_w_/2, lane_width)
{
    //m_Busy = false;
    //priv_nh_.param<std::string>("PATH_2FEATURES", PATH_2FEATURES, "");
    //priv_nh_.param<std::string>("PATH_30FEATURES", PATH_30FEATURES, "");

    cam_w = cam_w_;
    cam_h = cam_h_;
    proj_y_start = proj_y_start_;
    proj_image_h = proj_image_h_;
    proj_image_w = proj_image_w_;
    proj_image_w_half = proj_image_w/2;
    roi_top_w = roi_top_w_;
    roi_bottom_w = roi_bottom_w_;
    proj_image_horizontal_offset = proj_image_horizontal_offset_;

    head_time_stamp = ros::Time::now();

    m_LastValue = 0;
    read_images_ = nh.subscribe(nh_.resolveName("/camera/ground_image_ipmapped"), 1,&cLaneDetection::ProcessInput,this);

    publish_curvature = nh.advertise<std_msgs::Float32>("/lane_model/curvature", MY_ROS_QUEUE_SIZE);

    image_transport::ImageTransport image_transport(nh);
    
    realsense_rgb_image_pub = image_transport.advertiseCamera("/lane_model/lane_model_image", 1);

    if (!rgb_camera_info)
    {
        rgb_camera_info.reset(new sensor_msgs::CameraInfo());
        rgb_camera_info->width = proj_image_w;
        rgb_camera_info->height = proj_image_h+50;
    }
}

cLaneDetection::~cLaneDetection()
{
}

int cLaneDetection::Init()
{
	//firstFrame = True;
	imagecount = 0;
	m_LastValue = 0;
	return 1;
}

//re-initialize the whole system
void cLaneDetection::resetSystem()
{
    //model = LaneModel(false);
}

void cLaneDetection::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    // VideoInput
    //std::cout << "Hey, listen!" << std::endl;
    //ROS_INFO("CERTAINTY:");
    //ros::Time begin = ros::Time::now();

    try
    {
        ros::WallTime begin = ros::WallTime::now();

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        
        Mat image;
        image = cv_ptr->image.clone();


        //Rect(X,Y,Width,Height)
        Mat transformedImage = image(Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
            proj_y_start,proj_image_w,proj_image_h)).clone();
        Mat sobeledImage     = image(Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
            proj_y_start,proj_image_w,proj_image_h)).clone();
        Mat groundPlane      = image(Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
            proj_y_start,proj_image_w,proj_image_h)).clone();


		// cv_bridge::CvImage out_msg;
		// out_msg.header   = msg->header; // Same timestamp and tf frame as input image
		// out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
		// out_msg.image    = transformedImage; // Your cv::Mat
  		// publish_images_.publish(out_msg.toImageMsg());
        
        //create an output image for debugging
        //Mat generalOutputImage(300,800,CV_8UC3,Scalar(0,0,0));
        vector<Point2d> laneMarkings = detector.detect(transformedImage,sobeledImage,groundPlane);

        //---------------------- DEBUG OUTPUT LANE MARKINGS ---------------------------------//
        #ifdef PAINT_OUTPUT
            Mat transformedImagePaintable = transformedImage.clone();
            cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
            for(int i = 0;i < (int)laneMarkings.size();i++)
            {
                circle(transformedImagePaintable,laneMarkings.at(i),1,Scalar(0,0,255),-1);
            }

            Point2d p1(proj_image_w_half-(roi_bottom_w/2),proj_image_h-1);
            Point2d p2(proj_image_w_half+(roi_bottom_w/2),proj_image_h-1);
            Point2d p3(proj_image_w_half+(roi_top_w/2),0);
            Point2d p4(proj_image_w_half-(roi_top_w/2),0);
            line(transformedImagePaintable,p1,p2,Scalar(0,200,0));
            line(transformedImagePaintable,p2,p3,Scalar(0,200,0));
            line(transformedImagePaintable,p3,p4,Scalar(0,200,0));
            line(transformedImagePaintable,p4,p1,Scalar(0,200,0));
            cv::imshow("Lane markings", transformedImagePaintable);
            //cv::imshow("Original image", image);
            cv::waitKey(1);
        #endif
        //---------------------- END DEBUG OUTPUT LANE MARKINGS ------------------------------//
        //use the detected lane markings to find contours in the image
        Mat circleImage(proj_image_h,proj_image_w,CV_8UC1,Scalar(0));
        for(int i = 0;i < (int)laneMarkings.size();i++)
        {
            circle(circleImage,laneMarkings.at(i),3,Scalar(255),-1);
        }

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
        ContourModel cModel;
        bool midLaneFound = cModel.update(contours,laneMarkings,proj_image_w_half);
        vector<vector<Point2d> > nicelyGroupedPoints = cModel.points;

        if (midLaneFound)
        	ROS_ERROR("Found a midlane!");
        
        #ifdef PAINT_OUTPUT
                //---------------------- DEBUG OUTPUT CONTOURS ---------------------------------//
            Mat contourImagePaintable = transformedImage.clone();
            cvtColor(contourImagePaintable,contourImagePaintable,CV_GRAY2BGR);            
            //draw the detected contours on the output image
            for(int i = 0;i < (int)nicelyGroupedPoints.size();i++)
            {
                Scalar color(255,255,255);
                if(i == 0)color = Scalar(0,0,255);if(i == 1)color = Scalar(0,255,0);if(i == 2)color = Scalar(255,0,0);
                if(i == 3)color = Scalar(255,255,0);if(i == 4)color = Scalar(255,0,255);if(i == 5)color = Scalar(255,255,0);

                vector<Point2d> pointGroup = nicelyGroupedPoints.at(i);                
                for(int j = 0;j < (int)pointGroup.size();j++)
                {
                    Point2d currP = pointGroup.at(j);
                    currP.x += proj_image_w_half;
                    circle(contourImagePaintable,currP,1,color,-1);
                }
            }
            cv::imshow("Contours", contourImagePaintable);

            //---------------------- END DEBUG OUTPUT CONTOURS------------------------------//
        #endif
       //ROS_ERROR("Found: %d",midLaneFound);
        model.improvedUpdate(&nicelyGroupedPoints,midLaneFound);
        //ROS_ERROR("CERTAINTY: %d",model.certainty);
        // SEND OUT OLD LANE INFORMATION
        double curvature; //1/cm
        double distanceRightLane; //cm
        double angle; //rad
        bool isCurve;
        model.getCarState(&curvature,&distanceRightLane,&angle,&isCurve);
        //send is curve info:
        int stamp = 0;
        ros::WallTime end = ros::WallTime::now();
        ros::WallDuration d= end-begin;
        ROS_ERROR("time: %ld", d.toNSec()/1000000); 


        // ros::Time end = ros::Time::now();
        // ROS_ERROR("time: %d", ((end.nsec-begin.nsec)/1000000)); 
        
            //---------------------- DEBUG OUTPUT LANE MODEL---------------------------------//
            int carOffset = 50;
            Mat laneModelDrawing(proj_image_h+carOffset,proj_image_w,CV_8UC3,Scalar(0,0,0));
            Mat transformedImageCopy = transformedImage.clone();
            cvtColor(transformedImageCopy,transformedImageCopy,CV_GRAY2BGR);
            transformedImageCopy.copyTo(laneModelDrawing(Rect(0,carOffset,transformedImageCopy.cols,transformedImageCopy.rows)));
            model.getDebugImage(laneModelDrawing);
        #ifdef PAINT_OUTPUT
            cv::imshow("Lane model", laneModelDrawing);
             
           //---------------------- END DEBUG OUTPUT LANE MODEL------------------------------//
        #endif

        pubRealSenseRGBImageMsg(laneModelDrawing);

        /*cv_bridge::CvImage out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", laneModelDrawing);


        sensor_msgs::CompressedImage compressed;
        compressed.header = msg.header;
          compressed.format = msg.encoding;

          // Compression settings
          std::vector<int> params;
          params.resize(3, 0);

*/

        //out_msg.header   = std_msgs::Header();
        //out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
        //out_msg.image    = laneModelDrawing; // Your cv::Mat
        //publish_images.publish(out_msg.toImageMsg());

        std_msgs::Float32 curvMsg;
        curvMsg.data = curvature*100;
        publish_curvature.publish(curvMsg);

    } 
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}


void cLaneDetection::pubRealSenseRGBImageMsg(cv::Mat& rgb_mat)
{
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    rgb_img->header.seq = head_sequence_id;
    rgb_img->header.stamp = head_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    /*rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;*/

    realsense_rgb_image_pub.publish(rgb_img, rgb_camera_info);


    //save rgb img
//  static int count = 0;
//  count++;
//  if(count > 0)
//  {
//      struct timeval save_time;
//        gettimeofday( &save_time, NULL );
//        char save_name[256];
//        sprintf(save_name, "~/temp/realsense_rgb_%d.jpg", (int)save_time.tv_sec);
//        printf("\nsave realsense rgb img: %s\n", save_name);
//      cv::imwrite(save_name, rgb_mat);
//      count = 0;
//  }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetection");
    ros::NodeHandle nh;

    //std::string node_name = "/line_detection_node/";
    std::string node_name = ros::this_node::getName();

    ROS_ERROR("%s",node_name.c_str());


    int cam_w;
    int cam_h;
    int proj_y_start;
    int proj_image_h;
    int proj_image_w;
    int roi_top_w;
    int roi_bottom_w;
    int proj_image_horizontal_offset;
    int detector_size;
    int lane_width;
    std::string path_2features;
    std::string path_30features;

    //nh.param<std::string>("camera_name", camera_name, "/usb_cam/image_raw"); 
    nh.param<int>("cam_w", cam_w, 640);
    nh.param<int>("cam_h", cam_h, 480);
    nh.param<int>(node_name+"/proj_y_start", proj_y_start, 400);
    nh.param<int>(node_name+"/proj_image_h", proj_image_h, 40);
    nh.param<int>(node_name+"/proj_image_w", proj_image_w, 80);
    nh.param<int>(node_name+"/roi_top_w", roi_top_w, 30);
    nh.param<int>(node_name+"/roi_bottom_w", roi_bottom_w, 20);
    nh.param<int>(node_name+"/proj_image_horizontal_offset", proj_image_horizontal_offset, 0);
    nh.param<int>(node_name+"/detector_size", detector_size, 16);
    nh.param<int>(node_name+"/lane_width", lane_width, 17);
    nh.param<std::string>(node_name+"/path_2features", path_2features,
        "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_2features.txt");
    nh.param<std::string>(node_name+"/path_30features", path_30features,
        "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_30features.txt");

    cLaneDetection node(nh, cam_w, cam_h, proj_y_start, proj_image_h, proj_image_w,
        roi_top_w, roi_bottom_w, proj_image_horizontal_offset, detector_size, lane_width, path_2features, path_30features);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}