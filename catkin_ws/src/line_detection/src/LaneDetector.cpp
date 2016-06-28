//
//  LaneDetector.cpp
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
 * $Author:: Paul Bergmann $  $Date:: 2015-01-15 13:29:48#$ $Rev:: 26104  $*
 ***************************************************************************/

#include "LaneDetector.h"
#include <ros/package.h>

//#define PATH_2FEATURES "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_2features.txt"
//#define PATH_30FEATURES "/home/vena/Dropbox/lane_detection/catkin_ws/src/line_detection/src/strongClassifiers/classifier_30features.txt"


LaneDetector::LaneDetector(int size,Point p1,Point p2, int image_h_, int image_w_, int roi_top_w_, int roi_bottom_w_,
    std::string path_2features, std::string path_30features) : classifier_2features(path_2features,0.5),classifier_30features(path_30features,0.5)
{
    detectorSize = size;
    detectorSizeHalf = size/2;
    foe_1 = p1;
    foe_2 = p2;
    image_w = image_w_;
    image_h = image_h_;
    image_w_half = image_w/2;
    roi_top_w = roi_top_w_;
    roi_bottom_w = roi_bottom_w_;
}

vector<Point2d> LaneDetector::detect(Mat grayscaleImage,Mat sobeledImage,Mat groundplaneImage)
{
    vector<Point2d> detectedMarkers;
    
    int totalDetectors = 0;
    int detectorsEval  = 0;
    

    //sobel the input image
    /*Mat sobeledImage;

    Mat grad_x,grad_y;
    Mat abs_grad_x,abs_grad_y;

    Sobel(grayscaleImage,grad_x,grayscaleImage.depth(),1,0);
    convertScaleAbs(grad_x,abs_grad_x);

    Sobel(grayscaleImage,grad_y,grayscaleImage.depth(),0,1);
    convertScaleAbs(grad_y,abs_grad_y);

    addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,0,sobeledImage);*/

    vector<Point> checkContour;
    checkContour.push_back(Point(image_w_half-(roi_bottom_w/2),image_h-1));
    checkContour.push_back(Point(image_w_half+(roi_bottom_w/2),image_h-1));
    checkContour.push_back(Point(image_w_half+(roi_top_w/2),0));
    checkContour.push_back(Point(image_w_half-(roi_top_w/2),0));


    for(int i = foe_1.y;i < foe_2.y-detectorSize;i+=6)
    {
        for(int j = foe_1.x;j < foe_2.x-detectorSize;j+=1)
        {

            if(pointPolygonTest(checkContour, Point(j+detectorSizeHalf,i+detectorSizeHalf),false) < 0)
            {
                continue;
            }


            totalDetectors++;
            
            if(groundplaneImage.at<uchar>(i+detectorSizeHalf,j+detectorSizeHalf) == 0)continue;
            if(sobeledImage.at<uchar>(i+detectorSizeHalf,j+detectorSizeHalf) < 100)continue;

            Rect x(j,i,detectorSize,detectorSize);
            Mat detectorGray = grayscaleImage(x);
                    
            
            Mat integralImage;Mat rotatedIntegralImage;Mat dummy;
            cv::integral(detectorGray, integralImage, dummy, rotatedIntegralImage);
            int classifyResult = classifier_2features.classifyImage(integralImage, rotatedIntegralImage);

            if(classifyResult == 0)continue;
            
            classifyResult = classifier_30features.classifyImage(integralImage, rotatedIntegralImage);

            detectorsEval++;

            if(classifyResult == 1)
            {
                //imshow("CORRECT",detectorGray);
                //waitKey(1);

                //cout << "Center of marking: " << j+detectorSizeHalf << "/" << i+detectorSizeHalf << endl;
                
                Point2d newPoint(j+detectorSizeHalf,i+detectorSizeHalf);
                detectedMarkers.push_back(newPoint);
                detectedMarkers.push_back(newPoint+Point2d(0,2));
                detectedMarkers.push_back(newPoint+Point2d(0,-2));

                detectedMarkers.push_back(newPoint+Point2d(-1,0));
                detectedMarkers.push_back(newPoint+Point2d(-1,2));
                detectedMarkers.push_back(newPoint+Point2d(-1,-2));

                detectedMarkers.push_back(newPoint+Point2d(1,0));
                detectedMarkers.push_back(newPoint+Point2d(1,2));
                detectedMarkers.push_back(newPoint+Point2d(1,-2));
                //detectedMarkers.push_back(newPoint+Point2d(0,-2));
                //detectedMarkers.push_back(newPoint+Point2d(0,2));
            }
        }
    }
    
    //cout << detectorsEval << "/" << totalDetectors << endl;
    return detectedMarkers;
}
