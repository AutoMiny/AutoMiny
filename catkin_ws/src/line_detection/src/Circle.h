//
//  Circle.h
//  LaneDetectionExtended
//
//  Created by Paul on 31.01.15.
//  Copyright (c) 2015 Paul Bergmann. All rights reserved.
//

#ifndef __LaneDetectionExtended__Circle__
#define __LaneDetectionExtended__Circle__

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

class Circle{
    
public:
    
    Circle(Point2d p1,Point2d p2,Point2d p3);
    
    bool isValid;
    Point2d center;
    double radius;
    
private:
    
    bool isInvalidCombination(Point2d p1,Point2d p2,Point2d p3);
    int getCircle(Point2d p1, Point2d p2, Point2d p3);
    bool isCollinear(Point2d p1,Point2d p2,Point2d p3);



    
};

#endif /* defined(__LaneDetectionExtended__Circle__) */
