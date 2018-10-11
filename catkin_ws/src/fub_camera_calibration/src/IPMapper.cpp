
#include "fub_camera_calibration/IPMapper.h"
//#include <ros/ros.h>


Mat IPMapper::remap(Mat input)
{	
    Mat remappedImage(outputHeight,outputWidth,CV_8UC1,Scalar(0));
    
    for(int y = 0;y < outputHeight;y++)
    {
        for(int x = 0;x < outputWidth;x++)
        {
            Point toCopy = mappingMatrix(y,x);
            if(toCopy.x >= 0 && toCopy.y >= 0 && toCopy.y < input.rows && toCopy.x < input.cols)
            {
                remappedImage.at<uchar>(y,x) = input.at<uchar>(toCopy.y,toCopy.x);
            }
            else
            {
                remappedImage.at<uchar>(y,x) = 150;
            }
        }
    }

    
      //GaussianBlur(remappedImage, remappedImage, Size(5,5),1);
    
    return remappedImage;
}

IPMapper::IPMapper() {}

IPMapper::IPMapper(int ow, int oh, double _f_u, double _f_v, double _c_u, double _c_v, double deg, double _cam_h)
{
    outputWidth  = ow;
    outputHeight = oh;
    
    //CAMERA PARAMETERS:
    f_u = _f_u;//624.650635;//524.692545;                           //focal lense values (mm)
    f_v = _f_v;//626.987244;//524.692545;
    
    c_u = _c_u;//309.703230;//319.5;                           //camera optical center
    c_v = _c_v;//231.473613;//239.5;

    double pi = 3.1415926;
    //double deg = d//27;
    c_1 = cos(pi/180*deg);                             //cos(alpha : pitch angle),cos(beta : yaw angle)
    c_2 = 1.0;
    
    s_1 = sin(pi/180*deg);                             //sin(alpha : pitch angle),sin(beta : yaw angle)
    s_2 = 0.0;
    
    cam_h = _cam_h;//18;
    
    //init projection matrices
    T = (Mat_<double>(4,4) <<   -c_2/f_u, s_1*s_2/f_v, c_u*c_2/f_u-c_v*s_1*s_2/f_v-c_1*s_2, 0,
         s_2/f_u, s_1*c_1/f_v, -c_u*s_2/f_u-c_v*s_1*c_2/f_v-c_1*c_2, 0,
         0, c_1/f_v, -c_v*c_1/f_v+s_1, 0,
         0, -c_1/(f_v*cam_h), c_v*c_1/(cam_h*f_v)-s_1/cam_h, 0);
    T = cam_h * T;
    
    T_INV = (Mat_<double>(4,4) << f_u*c_2+c_u*c_1*s_2,c_u*c_1*c_2-s_2*f_u,-c_u*s_1,0,
             s_2*(c_v*c_1-f_v*s_1),c_2*(c_v*c_1-f_v*s_1),-f_v*c_1-c_v*s_1,0,
             c_1*s_2,c_1*c_2,-s_1,0,
             c_1*s_2,c_1*c_2,-s_1,0);
    
    mappingMatrix = Mat_<cv::Point>(outputHeight, outputWidth, cv::Point(0, 0));
    initMappingMatrix(&mappingMatrix);
}

void IPMapper::initMappingMatrix(Mat_<cv::Point>* scaledPointMatrix)
{
    int last_x,last_y=-1;
    int first_x=-1;
    int first_y=-1;
    Mat_<cv::Point> p=Mat_<cv::Point>(outputHeight, outputWidth, cv::Point(0, 0));

    Mat_<cv::Point> * pointMatrix=&p;

    for(int y = 0;y < pointMatrix->rows;y++)
    {
        for(int x = 0;x < pointMatrix->cols;x++)
        {
            Point toTransform(x-pointMatrix->cols/2,y);
            
            vector<Point> toMap;
            toMap.push_back(toTransform);
            vector<Point2d> result;
            invProjectPoints(&toMap, &result);


            if(result.at(0).x >= 0 && result.at(0).x < outputWidth && result.at(0).y >= 0 && result.at(0).y < outputHeight)
            {
                pointMatrix->at<Point>(y,x) = result.at(0);
                if (first_x==-1 && first_y==-1)
                {
                    first_x=x;
                    first_y=y;
                }
                if (x>last_x)
                    last_x=x;
                if (y>last_y)
                    last_y=y;
                if (x<first_x)
                    first_x=x;
                if (y<first_y)
                    first_y=y;            
            }
            else
            {
                pointMatrix->at<Point>(y,x) = Point2d(1000,1000);
            }

        }
    }
   
    if (last_y>0)
    {
        std::cout <<"first_x "<< first_x << ", last_x " << last_x << ". first_y "<< first_y << " last_y "<< last_y << "\n";
        double scale=std::min(double(outputWidth)/double(last_x-first_x),double(outputHeight)/double(last_y-first_y));
        std::cout << "scale:  "<< scale << "\n";
        for(int y = 0;y < scaledPointMatrix->rows;y++)
        {
            for(int x = 0;x < scaledPointMatrix->cols;x++)
            {
                scaledPointMatrix->at<Point>(y,x) = pointMatrix->at<Point>((y/scale+first_y),(x/scale+first_x));
            }
        }
    }
    else
    {
       scaledPointMatrix= pointMatrix;
    }
}

void IPMapper::projectPoints(vector<Point>* points,vector<Point2d>* result)
{
    //for each lane marking
    for(int i = 0;i < points->size();i++)
    {
        Point currPoint = points->at(i);
        Mat P_I = (Mat_<double>(4,1) << currPoint.x,currPoint.y,1,1);
        Mat P_G = T*P_I;
        P_G /= P_G.at<double>(3);
        
        Point2d transformedPoint(P_G.at<double>(0),P_G.at<double>(1));
        result->push_back(transformedPoint);
    }
}

void IPMapper::invProjectPoints(vector<Point>* points,vector<Point2d>* result)
{
    for(int i = 0;i < points->size();i++)
    {
        Point currPoint = points->at(i);
        Mat P_G = (Mat_<double>(4,1) << currPoint.x,currPoint.y,-cam_h,1);
        Mat P_I = T_INV * P_G;
        P_I = P_I / P_I.at<double>(3);
        Point2d invTransformedPoint(P_I.at<double>(0),P_I.at<double>(1));
        result->push_back(invTransformedPoint);
        
    }
}
