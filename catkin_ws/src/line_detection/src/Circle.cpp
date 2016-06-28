//
//  Circle.cpp
//  LaneDetectionExtended
//
//  Created by Paul on 31.01.15.
//  Copyright (c) 2015 Paul Bergmann. All rights reserved.
//

#include "Circle.h"

Circle::Circle(Point2d p1,Point2d p2,Point2d p3)
{
    int maxRadius = 9000000;
    
    if(isCollinear(p1, p2, p3)){
        p1.x += 1;
       // isValid = false;
      //  cout << "COLLINEAR" << endl;
      //  return;
    }
       
    if (!this->isInvalidCombination(p1, p2, p3) )				this->getCircle(p1, p2, p3);
    else if (!this->isInvalidCombination(p1, p3, p2) )		this->getCircle(p1, p3, p2);
    else if (!this->isInvalidCombination(p2, p1, p3) )		this->getCircle(p2, p1, p3);
    else if (!this->isInvalidCombination(p2, p3, p1) )		this->getCircle(p2, p3, p1);
    else if (!this->isInvalidCombination(p3, p2, p1) )		this->getCircle(p3, p2, p1);
    else if (!this->isInvalidCombination(p3, p1, p2) )		this->getCircle(p3, p1, p2);
    else {
        isValid = false;;
        return ;
    }
    
    if(radius < maxRadius)isValid = true;
    else isValid = false;
}

bool Circle::isCollinear(Point2d p1,Point2d p2,Point2d p3)
{
    if((p1.x*(p2.y-p3.y)+p2.x*(p3.y-p1.y)+p3.x*(p1.y-p2.y))<= 0.000001)return true;
    return false;
}

//check weather this point combination can be fitted => perpendicularity
bool Circle::isInvalidCombination(Point2d p1,Point2d p2,Point2d p3)
{
    double dy_1 = p2.y - p1.y;
    double dx_1 = p2.x - p1.x;
    double dy_2 = p3.y - p2.y;
    double dx_2 = p3.x - p2.x;
    
    if (abs(dx_1) <= 0.000000001 && abs(dy_2) <= 0.000000001){
        return false;
    }
    
    if (abs(dy_1) <= 0.0000001){
        return true;
    }
    else if (abs(dy_2) <= 0.0000001){
        return true;
    }
    else if (abs(dx_1)<= 0.000000001){
        return true;
    }
    else if (abs(dx_2)<= 0.000000001){
        return true;
    }
    else return false ;
    
    return true;
}

int Circle::getCircle(Point2d p1, Point2d p2, Point2d p3)
{
    double dy_1= p2.y - p1.y;
    double dx_1= p2.x - p1.x;
    double dy_2= p3.y - p2.y;
    double dx_2= p3.x - p2.x;
    
    if (abs(dx_1) <= 0.000000001 && abs(dy_2) <= 0.000000001){
        center.x= 0.5*(p2.x + p3.x);
        center.y= 0.5*(p1.y + p2.y);
        
        Point2d toRadiusP = p1-center;
        radius = sqrt(toRadiusP.x*toRadiusP.x + toRadiusP.y*toRadiusP.y) ;
        return 0;
    }
    
    // isInvalidCombination() assure that xDelta(s) are not zero
    double aSlope=dy_1/dx_1; //
    double bSlope=dy_2/dx_2;
    if (abs(aSlope-bSlope) <= 0.000000001){	// checking whether the given points are colinear.
        return -1;
    }
    
    // calc center
    center.x = (aSlope*bSlope*(p1.y - p3.y) + bSlope*(p1.x + p2.x)
                         - aSlope*(p2.x+p3.x) )/(2* (bSlope-aSlope) );
    center.y = -1*(center.x - (p1.x+p2.x)/2)/aSlope +  (p1.y+p2.y)/2;
    
    Point2d toRadiusP = p1-center;
    radius = sqrt(toRadiusP.x*toRadiusP.x + toRadiusP.y*toRadiusP.y) ;

    return 1;
}