
#ifndef __TestExtendedDetector__IPMapper__
#define __TestExtendedDetector__IPMapper__

#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

class IPMapper{
    
public:
    
    IPMapper(int outputWidth,int outputHeight, double f_u, double f_v, double c_u, double c_v, double deg, double cam_h);
    Mat remap(Mat input);
    //project points ground plane => image
    void invProjectPoints(vector<Point>* points,vector<Point2d>* result);
    
    
private:
    
    int outputWidth,outputHeight;
    
    //Project points image => ground plane
    void projectPoints(vector<Point>* points,vector<Point2d>* result);
    
 
    //init the mapping matrix to enable fast remapping of the image
    void initMappingMatrix(Mat_<cv::Point>* pointMatrix);
    
    //CAMERA PARAMETERS:
    double f_u;                             //focal lense values (mm)
    double f_v;
    
    double c_u;                             //camera optical center
    double c_v;
    
    double c_1;                             //cos(alpha : pitch angle),cos(beta : yaw angle)
    double c_2;
    
    double s_1;                             //sin(alpha : pitch angle),sin(beta : yaw angle)
    double s_2;
    
    double cam_h;                           //height of the camera above ground (cm)
    
    //Transformation Matrix T => camera view to top view and back
    Mat T,T_INV;
    
    Mat_<Point> mappingMatrix;

};

#endif /* defined(__TestExtendedDetector__IPMapper__) */
