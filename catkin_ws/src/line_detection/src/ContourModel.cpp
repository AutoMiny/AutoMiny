//
//  ContourModel.cpp
//  LaneDetection
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

#include "ContourModel.h"

//check weather to merge two contours based on the center points and directions
bool ContourModel::mergeContours(Point2d centre1,Point2d dir1,Point2d centre2,Point2d dir2)
{
    Point centreDir = centre2-centre1;
    
    //calc angle centre1/2 centreDir and threshold this to boolean
    
    double scalarProd1 = (centreDir.x*dir1.x+centreDir.y*dir1.y);
    double absDir1    = sqrt(dir1.x*dir1.x+dir1.y*dir1.y);
    double absCent    = sqrt(centreDir.x*centreDir.x+centreDir.y*centreDir.y);
    
    double cos1 = scalarProd1/(absDir1*absCent);
    
    double scalarProd2 = (centreDir.x*dir2.x+centreDir.y*dir2.y);
    double absDir2    = sqrt(dir2.x*dir2.x+dir2.y*dir2.y);
    
    double cos2 = scalarProd2/(absDir2*absCent);
    
    double angle1 = acos(cos1)*(180/3.1415926);
    double angle2 = acos(cos2)*(180/3.1415926);
    
    if(angle1 > 90) angle1 = 180-angle1;
    if(angle2 > 90) angle2 = 180-angle2;
    
    
    int angleThresh = 20;
    return (abs(angle1) < angleThresh && abs(angle2) < angleThresh);
}


//compare two contours
bool acompareCont(vector<Point> lContour, vector<Point> rContour)
{
    return contourArea(lContour) < contourArea(rContour);
}

bool ContourModel::update(vector<vector<Point> > contours,vector<Point2d>& originalPoints, int image_w_half)
{
    
    //step 1: find the larger contours to filter out some noise (area > thresh)
    vector<vector<Point> > largeContours;
    
    int areaThreshold = 130;
    for(int i = 0;i < (int)contours.size();i++)
    {
        vector<Point> currCont = contours.at(i);
        double area = contourArea(contours.at(i));
        if(area > areaThreshold)
        {
            largeContours.push_back(currCont);
        }
    }

    
    //step 2: for each larger contour: find the center of mass and the lane direction to group them
    vector<Point2d> mass_centers;
    vector<Point2d> line_directions;
    for(int i = 0;i < (int)largeContours.size();i++)
    {
        //calculate the line direction for each contour
        Vec4f lineParams;
        fitLine(largeContours.at(i), lineParams, CV_DIST_L2, 0, 0.01, 0.01);
        Point2d lineDirection(lineParams[0],lineParams[1]);
        line_directions.push_back(lineDirection);
        
        //calculate the mass center for each contour
        vector<Moments> contourMoments;
        Moments currMoments = moments(largeContours.at(i));
        double x_cent = currMoments.m10 / currMoments.m00;
        double y_cent = currMoments.m01 / currMoments.m00;
        Point2d mass_cent(x_cent,y_cent);
        mass_centers.push_back(mass_cent);    
    }

    //assert these vectors have same length:
    if(largeContours.size() != mass_centers.size())cout << "ERROR in ContourModel: massCenters.size != largeContours.size()" << endl;
    if(largeContours.size() != line_directions.size())cout << "ERROR in ContourModel: massCenters.size != largeContours.size()" << endl;

    //step 3: create the "mergeList": store for each contour weather it wants to merge with another one
    vector<vector<int> > mergelist;
    //merge contours based on center of mass and line direction
    for(int i = 0;i < (int)largeContours.size();i++)
    {
        vector<int> mergeWishes;
        
        Point2d currCenter = mass_centers.at(i);
        Point2d currDirection = line_directions.at(i);
        
        for(int j = i+1;j < (int)largeContours.size();j++)
        {
            Point2d compCenter = mass_centers.at(j);
            Point2d compDirection = line_directions.at(j);
            
            bool wantMerge = mergeContours(currCenter, currDirection, compCenter, compDirection);
            
            if(wantMerge)mergeWishes.push_back(j);
        }
        
        mergelist.push_back(mergeWishes);
    }

    //step 4: use the mergeList to create the final_mergelist which looks as follows:
    //[ [0,2,5] [3] [1] [4,6]] telling which contours should be merged together
    
    vector<vector<int> > final_mergelist;
    for(int i = 0;i < (int)largeContours.size();i++)
    {
        vector<int> temp;
        temp.push_back(i);
        final_mergelist.push_back(temp);
    }
    
    for(int i = 0;i < (int)largeContours.size();i++)
    {
        vector<int>* containerToPushTo = NULL;
        
        //step 1: find the container the contour i is in - note that this will always succeed so containerToPushTo wont stay NULL
        for(int j = 0;j < (int)final_mergelist.size();j++)
        {
            vector<int>* currContainer;
            currContainer = &final_mergelist.at(j);
            for(int k = 0;k < (int)final_mergelist.at(j).size();k++)
            {
                if(final_mergelist.at(j).at(k) == i)
                {
                    containerToPushTo = currContainer;
                }
            }
        }
        
        //step2: for each element to push: make sure it appears in the container
        for(int j = 0;j < (int)mergelist.at(i).size();j++)
        {
            int elemToMerge = mergelist.at(i).at(j);
            
            //if elemToMerge already appears in containerToPushTo => do nothing
            bool alreadyInContainer = false;
            for(int k = 0;k < (int)containerToPushTo->size();k++)
            {
                if(containerToPushTo->at(k) == elemToMerge)
                    alreadyInContainer = true;
            }
            
            //not inside: push the element and delete it from the old vector it was in
            if(!alreadyInContainer)
            {
                
                //delete it from the old container!!
                for(int k = 0;k < (int)final_mergelist.size();k++)
                {
                    for(int l = 0;l < (int)final_mergelist.at(k).size();l++)
                    {
                        //DEBUG IFS - ERASE LATER
                        if(k < 0 || k >= (int)final_mergelist.size())cout << "OVERFLOW IN 159::ContourModel" << endl;
                        if(l < 0 || l >= (int)final_mergelist.at(k).size())cout << "OVERFLOW IN 160::ContourModel" << endl;

                        if(final_mergelist.at(k).at(l) == elemToMerge)
                        {
                            //DEBUG IF- ERASE LATER
                            if(l < 0 || l >= (int)final_mergelist.at(k).size()) cout << "ERROR ContourModel 162" << endl;
                            final_mergelist.at(k).erase(final_mergelist.at(k).begin()+l);
                        }
                    }
                }
                
                //add it in the new container
                containerToPushTo->push_back(elemToMerge);
            }
            
        }
        
    }
    
    
    //step 5: merge the contours together
    vector< vector<vector<Point> > > mergedContours;
    
    for(int i = 0;i < (int)final_mergelist.size();i++)
    {
        vector<vector<Point> > currGrouping;
        for(int j = 0;j < (int)final_mergelist.at(i).size();j++)
        {
            vector<Point> currContour = largeContours.at(final_mergelist.at(i).at(j));
            currGrouping.push_back(currContour);
        }
        if(currGrouping.size() > 0)mergedContours.push_back(currGrouping);
        
    }


    //TRY TO FIND THE MIDDLE LANE

    vector<vector<Point> >          singleContours;
    vector<vector<vector<Point> > > multipleContours;

    for(int i = 0;i < (int)mergedContours.size();i++)
    {        
        vector<vector<Point> > currContGroup = mergedContours.at(i);
        if(currContGroup.size() == 1) singleContours.push_back(currContGroup.at(0));
        else if(currContGroup.size() > 1) multipleContours.push_back(currContGroup);                
    }

    //in this situation there is actually a chance to apply the middle lane extraction, otherwise the old procedure is applied
    if(multipleContours.size() == 1 && singleContours.size() <= 2 && singleContours.size() > 0)
    {

        //sort single contours by area
        std::sort(singleContours.begin(),singleContours.end(),acompareCont);
        vector<Point> largestSingleContour = singleContours.at(singleContours.size()-1);
        double areaLargestSingle = contourArea(largestSingleContour);

        vector<vector<Point> > middleContour = multipleContours.at(0);
        double areaMiddle = 0;
        bool validMid = true;
        for(int i = 0;i < (int)middleContour.size();i++)
        {
            double areaCurr = contourArea(middleContour.at(i));
            if(areaCurr > areaLargestSingle/2.0){
                validMid = false;
            }
            areaMiddle += contourArea(middleContour.at(i));
        }


        //if both contours have a certain size
        if(areaLargestSingle > 120 && areaMiddle > 120)
        {
            //MIDDLE LANE AND OTHER LANE FOUND => RETURN THE ESTIMATE

            //first argument will be the middle lane
            //second argument will be the other larger lane
            vector<vector<Point2d> > nicelyGroupedPoints;

            //1) --- MIDDLE LANE ---
            vector<Point2d> temp_result;
            for(int i = 0;i < (int)middleContour.size();i++)
            {
                vector<Point> currCont = middleContour.at(i);
                Rect bound = boundingRect(currCont);

                //visit every point in the bounding rect
                for(int y = bound.y;y < bound.y+bound.height;y++)
                {
                    for(int x = bound.x;x < bound.x+bound.width;x++)
                    {
                        if(pointPolygonTest(currCont, Point(x,y), false) >= 0)
                        {
                            temp_result.push_back(Point2d(x-image_w_half,y));
                        }
                    }
                }

            }

            nicelyGroupedPoints.push_back(temp_result);

            //2) --- OTHER LANE ---

            vector<Point2d> temp_result2;

            Rect bound = boundingRect(largestSingleContour);

            //visit every point in the bounding rect
            for(int y = bound.y;y < bound.y+bound.height;y++)
            {
                for(int x = bound.x;x < bound.x+bound.width;x++)
                {
                    if(pointPolygonTest(largestSingleContour, Point(x,y), false) >= 0)
                    {
                        temp_result2.push_back(Point2d(x-image_w_half,y));
                    }
                }
            }


            if(validMid)
            {
                nicelyGroupedPoints.push_back(temp_result2);
                points = nicelyGroupedPoints;
                return true; //middle lane estimate provided
            }
        }
    }

    //MIDDLE LANE WAS NOT FOUND



    //step 6: get the final result: the grouped points matching the contours
    //need to perform a inside contour check within the bounding rectangle of the contour for
    //each point in the bounding rectangle
    vector<vector<Point2d> > nicelyGroupedPoints;

    for(int i = 0;i < (int)mergedContours.size();i++)
    {
        vector<Point2d> temp_result;
        for(int j = 0;j < (int)mergedContours.at(i).size();j++)
        {
            vector<Point> currContour = mergedContours.at(i).at(j);
            Rect bound = boundingRect(currContour);

            //visit every point in the bounding rect
            for(int y = bound.y;y < bound.y+bound.height;y++)
            {
                for(int x = bound.x;x < bound.x+bound.width;x++)
                {
                    if(pointPolygonTest(currContour, Point(x,y), false) >= 0)
                    {
                        temp_result.push_back(Point2d(x-image_w_half,y));
                    }
                }
            }
        }

        if(temp_result.size() > 0)
        {
            nicelyGroupedPoints.push_back(temp_result);
        }
    }


    /*
    //step 6 (alternative): get the final result: the grouped points matching the contours
    //need to perform a inside contour check for the input points if in boundary rectangle of the contour
    vector<vector<Point2d> > nicelyGroupedPoints;

    for(int i = 0;i < mergedContours.size();i++)
    {

        vector<Point2d> temp_result;
        for(int j = 0;j < mergedContours.at(i).size();j++)
        {
            vector<Point> currContour = mergedContours.at(i).at(j);
            Rect bound = boundingRect(currContour);

            for(int k = 0;k < originalPoints.size();k++)
            {
                //check if within the contour:
                if(pointPolygonTest(currContour, originalPoints.at(k), false) >= 0)
                {
                    temp_result.push_back(Point2d(originalPoints.at(k).x-image_w_half, originalPoints.at(k).y));
                }
            }
        }

        if(temp_result.size() > 0)
        {
            nicelyGroupedPoints.push_back(temp_result);
        }
    }

    */


    points = nicelyGroupedPoints;
    return false; //everything as usual, no further information
}
