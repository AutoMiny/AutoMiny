//
//  IPMapper.h
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
    
    IPMapper(int outputWidth,int outputHeight);
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
