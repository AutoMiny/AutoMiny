//
//  Feature.h
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

#ifndef __TestExtendedDetector__Feature__
#define __TestExtendedDetector__Feature__

#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

class Feature{
    
public:
    Feature(int fType,int fx,int fy,int fw,int fh);
    Feature(const Feature& f);
    Feature();

    //evaluate the feature on the integral image
    int evaluate(Mat integralImage,Mat rotatedIntegralImage);

    //is this a valid feature given the detector size
    bool isValid(int detectorWidth,int detectorHeigth);
    
    //check Feature f1 == Feature f2
    bool equals(Feature f);
    
    //get private attributes
    int getType(){return type;}
    int getX(){return x;}
    int getY(){return y;}
    int getWidth(){return w;}
    int getHeight(){return h;}
    
    //print feature values
    void print();

    
private:
    int type;
    int x;
    int y;
    int w;
    int h;
    
    //calculate the sum of a (rotated) rectangle in an image
    int rectangleSum(Mat integralImage,int x,int y,int w,int h,bool rotate);
    
    //Edge features
    bool calculateFeature1(Mat integralImage,int* value);
    bool calculateFeature2(Mat integralImage,int* value);
    bool calculateFeature3(Mat integralImage,int* value);
    bool calculateFeature4(Mat integralImage,int* value);
    
    //Line features
    bool calculateFeature5 (Mat integralImage,int* value);
    bool calculateFeature6 (Mat integralImage,int* value);
    bool calculateFeature7 (Mat integralImage,int* value);
    bool calculateFeature8 (Mat integralImage,int* value);
    bool calculateFeature9 (Mat integralImage,int* value);
    bool calculateFeature10(Mat integralImage,int* value);
    bool calculateFeature11(Mat integralImage,int* value);
    bool calculateFeature12(Mat integralImage,int* value);
    
    //Center features
    bool calculateFeature13(Mat integralImage,int* value);
    bool calculateFeature14(Mat integralImage,int* value);
    
    //validity functions for each feature
    bool feature1Valid (int imageWidth,int imageHeigth);
    bool feature2Valid (int imageWidth,int imageHeigth);
    bool feature3Valid (int imageWidth,int imageHeigth);
    bool feature4Valid (int imageWidth,int imageHeigth);
    bool feature5Valid (int imageWidth,int imageHeigth);
    bool feature6Valid (int imageWidth,int imageHeigth);
    bool feature7Valid (int imageWidth,int imageHeigth);
    bool feature8Valid (int imageWidth,int imageHeigth);
    bool feature9Valid (int imageWidth,int imageHeigth);
    bool feature10Valid(int imageWidth,int imageHeigth);
    bool feature11Valid(int imageWidth,int imageHeigth);
    bool feature12Valid(int imageWidth,int imageHeigth);
    bool feature13Valid(int imageWidth,int imageHeigth);
    bool feature14Valid(int imageWidth,int imageHeigth);

};

#endif /* defined(__TestExtendedDetector__Feature__) */
