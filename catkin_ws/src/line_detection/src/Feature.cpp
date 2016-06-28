//
//  Feature.cpp
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

#include "Feature.h"

Feature::Feature(int fType,int fx,int fy,int fw,int fh)
{
    type = fType;
    x = fx;
    y = fy;
    w = fw;
    h = fh;
}

//copy constructor
Feature::Feature(const Feature& f)
{
    type = f.type;
    x = f.x;
    y = f.y;
    w = f.w;
    h = f.h;
}

//invalid feature
Feature::Feature()
{
    type = -1;
}


//checks Feature this == Feature f
bool Feature::equals(Feature f)
{
    return(type == f.type && x == f.x && y == f.y && w == f.w && h == f.h);
}

//this function takes a feature and the integral images and simply evaluates the feature by calling its corresponding function
int Feature::evaluate(Mat integralImage,Mat rotatedIntegralImage)
{
    int result = 0;
    switch(type){
        case 1:  calculateFeature1 (integralImage,&result);          return result;
        case 2:  calculateFeature2 (integralImage,&result);          return result;
        case 3:  calculateFeature3 (rotatedIntegralImage,&result);   return result;
        case 4:  calculateFeature4 (rotatedIntegralImage,&result);   return result;
        case 5:  calculateFeature5 (integralImage,&result);          return result;
        case 6:  calculateFeature6 (integralImage,&result);          return result;
        case 7:  calculateFeature7 (integralImage,&result);          return result;
        case 8:  calculateFeature8 (integralImage,&result);          return result;
        case 9:  calculateFeature9 (rotatedIntegralImage,&result);   return result;
        case 10: calculateFeature10(rotatedIntegralImage,&result);   return result;
        case 11: calculateFeature11(rotatedIntegralImage,&result);   return result;
        case 12: calculateFeature12(rotatedIntegralImage,&result);   return result;
        case 13: calculateFeature13(integralImage,&result);          return result;
        case 14: calculateFeature14(rotatedIntegralImage,&result);   return result;
    }
    return -1;
}

//calculate the sum of a rectangular area in any integral image
//using 4 control points to calculate the sum in constant time
//can calculate regular rectangles or rectangles turned by 45° in the image
int Feature::rectangleSum(Mat integralImage,int x,int y,int w,int h,bool rotate = false)
{
    //openCV adds 1 to the height and the width of the integral image => skip
    x++;
    y++;
    
    //calculate a regular rectangle
    if(!rotate)
    {
        int p1 = integralImage.at<int>(y-1,x-1);
        int p2 = integralImage.at<int>(y-1,x+w-1);
        int p3 = integralImage.at<int>(y+h-1,x-1);
        int p4 = integralImage.at<int>(y+h-1,x+w-1);
        
        return (p1+p4-p2-p3);
    }
    //calculate a rectangle turned by 45°
    else
    {
        //check for out of bounds
        bool isInvalidInput = (y+h-1 >= integralImage.rows) || (y+w-1 >= integralImage.rows) || (y+w+h-1 >= integralImage.rows) || (x-h < 0) || (x+w >= integralImage.cols) || (x-h+w >= integralImage.cols);
        if(isInvalidInput)return -1;
        
        int p1 = integralImage.at<int>(y-1,x);
        int p2 = integralImage.at<int>(y+h-1,x-h);
        int p3 = integralImage.at<int>(y+w-1,x+w);
        int p4 = integralImage.at<int>(y+w+h-1,x-h+w);
        
        return (p1+p4-p2-p3);
    }
}

bool Feature::calculateFeature1(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x,y,w/2,h);
    
    *value = area1-2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature2(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x,y,w,h/2);
    
    *value = area1-2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature3(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x,y,w/2,h,true);
    
    *value = area1-2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature4(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x,y,w,h/2,true);
    
    *value = -area1+2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature5(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x+w/3,y,w/3,h);
    
    *value = -area1+3*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature6(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x+w/4,y,w/2,h);
    
    *value = -area1+2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature7(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x,y+h/3,w,h/3);
    
    *value = -area1+3*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature8(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x,y+h/4,w,h/2);
    
    *value = -area1+2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature9(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x-h/3,y+h/3,w,h/3,true);
    
    *value = -area1+3*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature10(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x-h/4,y+h/4,w,h/2,true);
    
    *value = -area1+2*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature11(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x+w/3,y+w/3,w/3,h,true);
    
    *value = -area1+3*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature12(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x+w/4,y+w/4,w/2,h,true);
    
    *value = -area1+3*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature13(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h);
    int area2 = rectangleSum(integralImage,x+w/3,y+h/3,w/3,h/3);
    
    *value = -area1+9*area2;
    return (area1 >= 0);
}

bool Feature::calculateFeature14(Mat integralImage,int* value)
{
    int area1 = rectangleSum(integralImage,x,y,w,h,true);
    int area2 = rectangleSum(integralImage,x,y+2*h/3,w/3,h/3,true);
    
    *value = -area1+9*area2;
    return (area1 >= 0);
}

bool Feature::isValid(int imageWidth,int imageHeigth)
{
    switch(type){
        case 1:   return feature1Valid (imageWidth, imageHeigth);
        case 2:   return feature2Valid (imageWidth, imageHeigth);
        case 3:   return feature3Valid (imageWidth, imageHeigth);
        case 4:   return feature4Valid (imageWidth, imageHeigth);
        case 5:   return feature5Valid (imageWidth, imageHeigth);
        case 6:   return feature6Valid (imageWidth, imageHeigth);
        case 7:   return feature7Valid (imageWidth, imageHeigth);
        case 8:   return feature8Valid (imageWidth, imageHeigth);
        case 9:   return feature9Valid (imageWidth, imageHeigth);
        case 10:  return feature10Valid(imageWidth, imageHeigth);
        case 11:  return feature11Valid(imageWidth, imageHeigth);
        case 12:  return feature12Valid(imageWidth, imageHeigth);
        case 13:  return feature13Valid(imageWidth, imageHeigth);
        case 14:  return feature14Valid(imageWidth, imageHeigth);
    }
    return false;
}

//the featureXValid functions are used to check wether a feature combination of (x,y,w,h) can be evaluated on a detecor
//they are later used to determine all available features in a simple manner
//basically they simply check out of bounds conditions of rectangles / rotated rectangles
bool Feature::feature1Valid(int imageWidth,int imageHeigth)
{
    if(w%2 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature2Valid(int imageWidth,int imageHeigth)
{
    if(h%2 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature3Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;
    imageHeigth++;
    x++;
    y++;
    
    bool isInvalidInput = (w%2 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

bool Feature::feature4Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;
    imageHeigth++;
    x++;
    y++;
    
    bool isInvalidInput = (h%2 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

bool Feature::feature5Valid(int imageWidth,int imageHeigth)
{
    if(w%3 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature6Valid(int imageWidth,int imageHeigth)
{
    if(w%4 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature7Valid(int imageWidth,int imageHeigth)
{
    if(h%3 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature8Valid(int imageWidth,int imageHeigth)
{
    if(h%4 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature9Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;imageHeigth++;x++;y++;
    
    bool isInvalidInput = (h%3 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

bool Feature::feature10Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;imageHeigth++;x++;y++;
    
    bool isInvalidInput = (h%4 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

bool Feature::feature11Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;imageHeigth++;x++;y++;
    
    bool isInvalidInput = (w%3 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

bool Feature::feature12Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;imageHeigth++;x++;y++;
    
    bool isInvalidInput = (w%4 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

bool Feature::feature13Valid(int imageWidth,int imageHeigth)
{
    if(w != h || h%3 != 0 || x+w > imageWidth || y+h > imageHeigth)return false;
    return true;
}

bool Feature::feature14Valid(int imageWidth,int imageHeigth)
{
    imageWidth++;imageHeigth++;x++;y++;
    
    bool isInvalidInput = (w != h || w%3 != 0) || (y+h-1 >= imageHeigth) || (y+w-1 >= imageHeigth) || (y+w+h-1 >= imageHeigth) || (x-h < 0) || (x+w >= imageWidth) || (x-h+w >= imageWidth);
    
    return !isInvalidInput;
}

void Feature::print()
{
    cout << "(" << type << "," << x << "," << y << "," << w << "," << h << ")" << endl;
}
