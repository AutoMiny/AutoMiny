#include <fub_camera_calibration/FubCameraStaticCalibration.h>

FubCameraStaticCalibration::FubCameraStaticCalibration(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it(nh)
{
    path = ros::package::getPath("fub_camera_calibration");
    image_topic_name = priv_nh_.param<std::string>("image", "/camera/color");
    file_name = priv_nh_.param<std::string>(path+"file_name", path+"/cfg/CameraParams.xml");
    file_3d_points = priv_nh_.param<std::string>(path+"file_3d_points", path+"/cfg/Object3DPoints.xml");
    debug = priv_nh_.param<bool>("debug", true);
    bi_gray_max = priv_nh_.param<int>("bi_gray_max", 255);
    bi_gray_min = priv_nh_.param<int>("bi_gray_min", 210);
    y_seg = priv_nh_.param<int>("y_seg", 50);
    x_seg = priv_nh_.param<int>("x_seg", 100);
    offset_y = priv_nh_.param<int>("offset_y", 320);
    offset_x = priv_nh_.param<int>("offset_x", 200);
    

    camera_params.intrinsicMatrix = (Mat1d(3, 3) << 616.9697, 0, 323.7197, 0, 617.4253, 244.3983, 0, 0, 1);
    camera_params.distCoeffs=(Mat1d(4, 1) << 0,0,0,0);
    camera_params.pitch=12;
    camera_params.height=18;
    objectPoints = Generate3DPoints();

    image_pub_gray = it.advertise("/reading_sensors/gray_img", 1);
    image_pub_bi_gray = it.advertise("/reading_sensors/binarized_gray_img", 1);
    sub_image = it.subscribe(image_topic_name+"/image_raw", 1,&FubCameraStaticCalibration::imageCallback,this);

    //sub_camera_info = it.subscribe(image_topic_name+"/camera_info", 1,&FubCameraStaticCalibration::cameraInfoCallback,this);

}
void FubCameraStaticCalibration::static_calibration()
{
    
    gray(cv_ptr->image);
    ROS_INFO("published croped image");

    Mat bi_gray_img = bi_gray(cv_ptr->image);

    std::vector<cv::Point2f> imagePoints=Generate2DPoints(bi_gray_img);

    std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);

    cv::solvePnP(objectPoints, imagePoints, camera_params.intrinsicMatrix, camera_params.distCoeffs, rvec, tvec);
    cv::Mat rmat_(3,3,cv::DataType<double>::type);

    // std::vector<cv::Point2f> projectedPoints;
    // cv::projectPoints(objectPoints, rvec, tvec, intrinsicMatrix, distCoeffs, projectedPoints);
    // for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    // {
    //     std::cout << "Image point: " << imagePoints[i] << " ~= " << projectedPoints[i] << std::endl;
    // }

    Rodrigues(rvec,rmat_);
    cv::Mat proj_matrix = Mat::zeros(3,4,CV_32F);
    rmat_.copyTo(proj_matrix.colRange(0,3).rowRange(0,3));
    cv::Mat cameraMatrix, rotMatrix, transVect, rotMatrX, rotMatrY, rotMatrZ, eulerAngles;
    cv::decomposeProjectionMatrix(proj_matrix, cameraMatrix, rotMatrix, transVect, rotMatrX, rotMatrY, rotMatrZ, eulerAngles);
    std::cout << eulerAngles.at<double>(0) << " "<< eulerAngles.at<double>(1)<< " "<<eulerAngles.at<double>(2) <<std::endl;
    // camera_params.=calculateEuler(rmat_);
    camera_params.pitch= -eulerAngles.at<double>(1);
    writeToFile();



}
double FubCameraStaticCalibration::calculateEuler(cv::Mat rmat)
{
    float sy = sqrt(rmat.at<double>(0,0) * rmat.at<double>(0,0) +  rmat.at<double>(1,0) * rmat.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float roll, pitch, yaw;
    if (!singular)
    {
        roll = -atan2(rmat.at<double>(2,1) , rmat.at<double>(2,2));
        pitch = -atan2(-rmat.at<double>(2,0), sy);
        yaw = -atan2(rmat.at<double>(1,0), rmat.at<double>(0,0));
    }
    else
    {
        roll = -atan2(-rmat.at<double>(1,2), rmat.at<double>(1,1));
        pitch = -atan2(-rmat.at<double>(2,0), sy);
        yaw = 0;
    }
    std::cout << "roll: " << roll*180/3.1416 << std::endl;
    std::cout << "pitch: " << pitch*180/3.1416 << std::endl;
    std::cout << "yaw: " << yaw*180/3.1416 << std::endl;
    return (pitch*180/3.1416);

}
std::vector<cv::Point2f> FubCameraStaticCalibration::Generate2DPoints(cv::Mat img)
{
    std::vector<cv::Point2f> points;

    float x,y;
    /*look for first point*/
    bool breaking=true;
    for (int i = 0; i < y_seg && breaking; i++)
            for (int j = 0; j < x_seg; j++)
            {

                if (img.at<uchar>(i,j) >= 200)
                {
                    x = j;
                    y = i;
                    break;
                }
            }
    points.push_back(cv::Point2f(x,y));
    breaking=true;
    for (int i = y_seg; i < y_seg*2&& breaking; i++)
            for (int j = 0; j < x_seg; j++)
            {
                if (img.at<uchar>(i,j) >= 200)
                {
                    x = j;
                    y = i;
                    break;
                }
            }
    points.push_back(cv::Point2f(x,y));
    breaking=true;

    for (int i = y_seg*2; i < y_seg*3&& breaking; i++)
            for (int j = 0; j < x_seg; j++)
            {
                if (img.at<uchar>(i,j) >= 200)
                {
                x = j;
                y = i;
                break;
                }
            }
    points.push_back(cv::Point2f(x,y));
    breaking=true;

    for (int i = 0; i < y_seg&& breaking; i++)
            for (int j = x_seg; j < x_seg*2; j++)
            {
                if (img.at<uchar>(i,j) >= 200)
                {
                x = j;
                y = i;
                break;
                }
            }
    points.push_back(cv::Point2f(x,y));
    breaking=true;

    for (int i = y_seg; i < y_seg*2&& breaking; i++)
            for (int j = x_seg; j < x_seg*2; j++)
            {
                if (img.at<uchar>(i,j) >= 200)
                {
                x = j;
                y = i;
                break;
                }
            }
    points.push_back(cv::Point2f(x,y));
    breaking=true;

    for (int i = y_seg*2; i < y_seg*3&& breaking; i++)
            for (int j = x_seg; j < x_seg*2; j++)
            {
                if (img.at<uchar>(i,j) >= 200)
                {
                x = j;
                y = i;
                break;
                }
            }
    points.push_back(cv::Point2f(x,y));

    for(unsigned int i = 0; i < points.size(); ++i)
      {
      std::cout << points[i] << std::endl;
      }

    return points;
}


std::vector<cv::Point3f> FubCameraStaticCalibration::Generate3DPoints()
{
    std::cout << "Generate3DPoints"<<std::endl;
    std::vector <fub_modelcar_tools::Object3DPoints> points;
    std::vector<cv::Point3f> points_;
    restoreXML(points,file_3d_points.c_str());

    for(int i = 0; i < points.size(); ++i)
    {
        std::cout << points.at(i).point3d << std::endl;
        points_.push_back(points.at(i).point3d);
    }

    return points_;
}

void FubCameraStaticCalibration::writeToFile()
{
    std::vector <fub_modelcar_tools::CameraCalibrationData> cameraParam;
    cameraParam.push_back(camera_params);
    // save data to archive
    saveXML(cameraParam,file_name.c_str());
//     std::ofstream file;
//     file.open(file_name.c_str());
//     file << "intrinsicMatrix:" << "\n" <<intrinsicMatrix << "\n";
//     file << "distCoeffs:" << "\n" <<distCoeffs << "\n";
//     file << "pitch: " << pitch <<"\n";
//     file << "height: " << height<<"\n";
//     file.close();
}
void FubCameraStaticCalibration::gray (cv::Mat img)
{
    Mat gray (img.size(), CV_8UC1);
    cvtColor(img, gray, CV_BGR2GRAY);
    sensor_msgs::ImagePtr cv_ptr_gray;
    cv_ptr_gray = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
    image_pub_gray.publish(cv_ptr_gray);
}

cv::Mat FubCameraStaticCalibration::bi_gray (cv::Mat img)
{
    Mat gray (img.size(), CV_8UC1);
    cvtColor(img, gray, CV_BGR2GRAY);

    cv::Mat bn = gray(Rect(offset_x, offset_y, x_seg*2, y_seg*3));
    ROS_INFO_STREAM("bn.size(): "<< bn.size());

    threshold(bn, bn, bi_gray_min, bi_gray_max, CV_THRESH_BINARY_INV);
    sensor_msgs::ImagePtr cv_ptr;
    cv_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", bn).toImageMsg();
    image_pub_bi_gray.publish(cv_ptr);
    return bn;
}


