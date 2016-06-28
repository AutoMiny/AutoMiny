//
//  LaneDetector.h
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

#ifndef __TestExtendedDetector__LaneDetector__
#define __TestExtendedDetector__LaneDetector__

#include <stdio.h>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include "StrongClassifier.h"

using namespace std;
using namespace cv;

class LaneDetector{
public:
    LaneDetector(int size,Point foe_p1,Point foe_p2, int image_h_, int image_w_, int roi_top_w_,
    	int roi_bottom_w_, std::string path_2features, std::string path_30features);
    vector<Point2d> detect(Mat grayscaleImage, Mat sobeledImage, Mat groundplaneImage);
    
private:
	std::string classifier_file_path;
    int detectorSize;
    int detectorSizeHalf;
    int image_w;
    int image_h;
    int image_w_half;
    int roi_top_w;
    int roi_bottom_w;
    Point foe_1;
    Point foe_2;
    StrongClassifier classifier_2features;
    StrongClassifier classifier_30features;
};

#endif /* defined(__TestExtendedDetector__LaneDetector__) */
