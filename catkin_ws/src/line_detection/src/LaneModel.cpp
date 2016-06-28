
//
//  LaneModel.cpp
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

#include "LaneModel.h"

#define USE_HISTORY_SIZE 2

LaneModel::LaneModel(bool doReject, int image_w_half_, int lane_width_)
{
    image_w_half = image_w_half_;
    laneWidth = lane_width_;
    initialize(doReject,false);
}

int LaneModel::getCurrentLane()
{
    if(lockRightLane)return 0;
    else return 1;
}

void LaneModel::initialize(bool doReject,bool doTurnLeft)
{
//    cout << "LD init" << endl;
    certainty = 0;

    rejectFirstFrames = doReject;

    counter = 0;
    yAcc = 0.0;
    angAcc = 0.0;
    badFitNumber = 0;

    streetHistoryMutex = PTHREAD_MUTEX_INITIALIZER;

    //laneWidth = 17;

    a_hist.clear();
    b_hist.clear();
    c_hist.clear();

    if(!doTurnLeft)
    {
        a_hist.push_back(0);
        b_hist.push_back(0);
        c_hist.push_back(0);
    }
    else
    {
        a_hist.push_back(0);
        b_hist.push_back(0);
        c_hist.push_back(-10);
    }

    a_hist_grouping.clear();
    b_hist_grouping.clear();
    c_hist_grouping.clear();
    validity_y_begin.clear();
    validity_y_end.clear();

    a_hist_grouping.push_back(0);
    b_hist_grouping.push_back(0);
    c_hist_grouping.push_back(0);

    validity_y_begin.push_back(50);
    validity_y_end.push_back(100);

    streetHistory.clear();

    for(int i = -80;i < 35;i++)
    {
        streetHistory.push_back(Point2d(-1,i));
        streetHistory.push_back(Point2d(0,i));
        streetHistory.push_back(Point2d(1,i));
    }

    lockRightLane = true;

    isCurveCounter = 0;
    
    modelLocked  = false;
}

void LaneModel::changeLane()
{
    pthread_mutex_lock(&streetHistoryMutex);

    //get the current lane estimate
    double a_avg = 0.0;
    double b_avg = 0.0;
    double c_avg = 0.0;

    double validity_begin = 2000;
    double validity_end   = -2000;

    //map the grouping parabolas to the other lane, get min max validity values
    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        a_avg += a_hist_grouping.at(i);
        b_avg += b_hist_grouping.at(i);
        c_avg += c_hist_grouping.at(i);

        if(validity_y_begin.at(i) < validity_begin) validity_begin = validity_y_begin.at(i);
        if(validity_y_end.at(i) > validity_end)     validity_end    = validity_y_end.at(i);
    }

    a_avg /= a_hist_grouping.size();
    b_avg /= b_hist_grouping.size();
    c_avg /= c_hist_grouping.size();


    double mid_value = (validity_begin+validity_end)/2.0;

    Point2d p1(a_avg*validity_begin*validity_begin+b_avg*validity_begin+c_avg,validity_begin);
    Point2d p2(a_avg*mid_value*mid_value+b_avg*mid_value+c_avg,mid_value);
    Point2d p3(a_avg*validity_end*validity_end+b_avg*validity_end+c_avg,validity_end);

    //Map the three points
    double m_p1 = 2*a_avg*validity_begin+b_avg;
    double m_p2 = 2*a_avg*mid_value+b_avg;
    double m_p3 = 2*a_avg*validity_end+b_avg;

    Point2d dirVec1(10,-10*m_p1);
    Point2d dirVec2(10,-10*m_p2);
    Point2d dirVec3(10,-10*m_p3);

    double len_1 = sqrt(100+100*m_p1*m_p1);
    double len_2 = sqrt(100+100*m_p2*m_p2);
    double len_3 = sqrt(100+100*m_p3*m_p3);

    dirVec1.x /= len_1;dirVec1.y /= len_1;
    dirVec2.x /= len_2;dirVec2.y /= len_2;
    dirVec3.x /= len_3;dirVec3.y /= len_3;

    if(lockRightLane)
    {
        dirVec1 *= -laneWidth;
        dirVec2 *= -laneWidth;
        dirVec3 *= -laneWidth;
    }
    else
    {
        dirVec1 *= laneWidth;
        dirVec2 *= laneWidth;
        dirVec3 *= laneWidth;
    }

    p1 += dirVec1;
    p2 += dirVec2;
    p3 += dirVec3;

    //solve parabola parameters
    Mat A = (Mat_<double>(3,3) <<   p1.y*p1.y,p1.y,1,
                                    p2.y*p2.y,p2.y,1,
                                    p3.y*p3.y,p3.y,1);

    Mat b = (Mat_<double>(3,1) <<   p1.x,p2.x,p3.x);

    Mat sol;

    solve(A, b, sol);

    double aFit = sol.at<double>(0,0);
    double bFit = sol.at<double>(1,0);
    double cFit = sol.at<double>(2,0);

    a_hist_grouping.clear();
    b_hist_grouping.clear();
    c_hist_grouping.clear();
    validity_y_begin.clear();
    validity_y_end.clear();

    a_hist_grouping.push_back(aFit);
    b_hist_grouping.push_back(bFit);
    c_hist_grouping.push_back(cFit);
    validity_y_begin.push_back(validity_begin);
    validity_y_end.push_back(validity_end);

    //mark lane switch in model
    if(lockRightLane) lockRightLane = false;
    else              lockRightLane = true;

    pthread_mutex_unlock(&streetHistoryMutex);

}


//use the middle lane to map the points and see if it makes sense according to the old model
void LaneModel::groupLaneMappingMidFound(vector<PointSet>* groups)
{
    PointSet* middleLaneSet = &groups->at(0);
    PointSet* otherLaneSet = &groups->at(1);

    //find min y point in middleLane and other lane set to figure out which lane the other lane belongs to
    Point2d min_y_mid(0,10000);
    Point2d min_y_other(0,10000);

    for(int i = 0;i < (int)middleLaneSet->consensusPoints.size();i++)
    {
        Point2d currP  = *middleLaneSet->consensusPoints.at(i);
        if(currP.y < min_y_mid.y)
        {
            min_y_mid = currP;
        }
    }

    for(int i = 0;i < (int)otherLaneSet->consensusPoints.size();i++)
    {
        Point2d currP  = *otherLaneSet->consensusPoints.at(i);
        if(currP.y < min_y_other.y)
        {
            min_y_other = currP;
        }
    }

    bool isLeft = false;
    if(min_y_other.x < min_y_mid.x)
    {
        isLeft = true;
    }

    //get a parabola estimate and check if it roughly fits with the last one (except for certainty < 10, then just overwrite)

    //use the middle lane for fitting
    if(middleLaneSet->consensusPoints.size() > otherLaneSet->consensusPoints.size())
    {
        for(int i = 0;i < (int)middleLaneSet->consensusPoints.size();i++)
        {
            Point2d temp = *middleLaneSet->consensusPoints.at(i);
            Point2d currP(middleLaneSet->a*temp.y*temp.y+middleLaneSet->b*temp.y+middleLaneSet->c,temp.y);

            //derivative
            double m;
            m = 2*middleLaneSet->a*currP.y+middleLaneSet->b;

            Point2d dirVec(10*m,10);
            Point2d rect_dir(10,-10*m);

            double len = sqrt(100+100*m*m);
            rect_dir.x /= len/(laneWidth/2);
            rect_dir.y /= len/(laneWidth/2);

            Point2d laneEstimation(currP.x+rect_dir.x,currP.y+rect_dir.y);

            currentStreetEstimate.push_back(laneEstimation);
        }
    }
    //use the other lane for fitting
    else
    {
        //left lane?
        if(isLeft)
        {
            for(int i = 0;i < (int)otherLaneSet->consensusPoints.size();i++)
            {
                Point2d temp = *otherLaneSet->consensusPoints.at(i);
                Point2d currP(otherLaneSet->a*temp.y*temp.y+otherLaneSet->b*temp.y+otherLaneSet->c,temp.y);

                double m = 2*otherLaneSet->a*currP.y + otherLaneSet->b;
                Point2d dirVec(10*m,10);
                Point2d rect_dir(10,-10*m);

                double len = sqrt(100+100*m*m);
                rect_dir.x /= len/(1.5*laneWidth);
                rect_dir.y /= len/(1.5*laneWidth);

                Point2d laneEstimation(currP.x+rect_dir.x,currP.y+rect_dir.y);

                currentStreetEstimate.push_back(laneEstimation);
            }
        }
        //right lane
        else
        {
            for(int i = 0;i < (int)otherLaneSet->consensusPoints.size();i++)
            {
                //Point2d currP = *otherLaneSet->consensusPoints.at(i);
                Point2d temp = *otherLaneSet->consensusPoints.at(i);
                Point2d currP(otherLaneSet->a*temp.y*temp.y+otherLaneSet->b*temp.y+otherLaneSet->c,temp.y);

                //derivative
                double m;
                m = 2*otherLaneSet->a*currP.y+otherLaneSet->b;


                Point2d dirVec(10*m,10);
                Point2d rect_dir(10,-10*m);

                double len = sqrt(100+100*m*m);
                rect_dir.x /= len/(laneWidth/2);
                rect_dir.y /= len/(laneWidth/2);

                Point2d laneEstimation(currP.x-rect_dir.x,currP.y-rect_dir.y);

                currentStreetEstimate.push_back(laneEstimation);
            }
        }
    }

}

//use the old history to map the points
void LaneModel::modelGroupLaneMapping(vector<PointSet>* groups)
{
    vector<double> groupingParams = getGroupingParabola();
    double a = groupingParams.at(0);
    double b = groupingParams.at(1);
    double c = groupingParams.at(2);

    for(int i = 0;i < (int)groups->size();i++)
    {
        PointSet* currSet = &groups->at(i);

        int min_y = 10000;
        int max_y = -10000;
        //find out min y,max y
        for(int j = 0;j < (int)currSet->consensusPoints.size();j++)
        {
            Point2d currPoint = *currSet->consensusPoints.at(j);
            if(currPoint.y > max_y)max_y = currPoint.y;
            if(currPoint.y < min_y)min_y = currPoint.y;
        }

        /*
        int closePoints = 0;

        for(int j = 0;j < (int)currSet->consensusPoints.size();j++)
        {
            Point2d currPoint = *currSet->consensusPoints.at(j);
            if(currPoint.y < min_y + 0.2*(max_y-min_y))closePoints++;
        }*/

        int right_lane_votes = 0;
        int mid_lane_votes = 0;
        int left_lane_votes = 0;

        for(int j = 0;j < (int)currSet->consensusPoints.size();j++)
        {
            //Point2d currPoint = *currSet->consensusPoints.at(j);
            Point2d temp = *currSet->consensusPoints.at(i);
            Point2d currPoint(currSet->a*temp.y*temp.y+currSet->b*temp.y+currSet->c,temp.y);



            //go from y values from [y-30,y+30] for each point in 5 px steps and check for min distance to parabola model
            double minSquareDistance = 99999999;
            for(int k = currPoint.y-30;k < currPoint.y+30;k += 2)
            {
                double lane_x = a*pow(k,2)+b*k+c;
                double squareDistance = sqrt((lane_x-currPoint.x)*(lane_x-currPoint.x)+(k-currPoint.y)*(k-currPoint.y));
                if(squareDistance < minSquareDistance)minSquareDistance = squareDistance;
            }

            double lane_x = a*currPoint.y*currPoint.y + b*currPoint.y + c;


            if(currPoint.x >= lane_x && (minSquareDistance) < laneWidth)
            {
                if(currPoint.y < min_y+0.2*(max_y-min_y)){right_lane_votes++;}
                right_lane_votes++;
            }

            else if(currPoint.x < lane_x && (minSquareDistance) < laneWidth)
            {
                if(currPoint.y < min_y+0.2*(max_y-min_y)){mid_lane_votes++;}
                mid_lane_votes++;
            }

            else if(currPoint.x < lane_x && (minSquareDistance) >= laneWidth && (minSquareDistance) < 2*laneWidth)
            {
                if(currPoint.y < min_y+0.2*(max_y-min_y)){left_lane_votes++;}
                left_lane_votes++;
            }
        }

        //this set was classified to be a right lane
        if(right_lane_votes >= 0.5*currSet->consensusPoints.size())
       // if(right_weights >= 0.3 && right_lane_votes > 5)
        {
            for(int i = 0;i < (int)currSet->consensusPoints.size();i++)
            {
                //Point2d currP = *currSet->consensusPoints.at(i);
                Point2d temp = *currSet->consensusPoints.at(i);
                Point2d currP(currSet->a*temp.y*temp.y+currSet->b*temp.y+currSet->c,temp.y);

                //derivative
                double m;
                m = 2*currSet->a*currP.y+currSet->b;


                Point2d dirVec(10*m,10);
                Point2d rect_dir(10,-10*m);

                double len = sqrt(100+100*m*m);
                rect_dir.x /= len/(laneWidth/2);
                rect_dir.y /= len/(laneWidth/2);

                Point2d laneEstimation(currP.x-rect_dir.x,currP.y-rect_dir.y);

                currentStreetEstimate.push_back(laneEstimation);
            }
        }

        //this set was classified to be a mid lane
        else if(mid_lane_votes >= 0.5*currSet->consensusPoints.size())
       // else if(mid_weights >= 0.3 && mid_lane_votes > 5)
        {
            for(int i = 0;i < (int)currSet->consensusPoints.size();i++)
            {
                //Point2d currP = *currSet->consensusPoints.at(i);
                Point2d temp = *currSet->consensusPoints.at(i);
                Point2d currP(currSet->a*temp.y*temp.y+currSet->b*temp.y+currSet->c,temp.y);

                //derivative
                double m;
                m = 2*currSet->a*currP.y+currSet->b;

                Point2d dirVec(10*m,10);
                Point2d rect_dir(10,-10*m);

                double len = sqrt(100+100*m*m);
                rect_dir.x /= len/(laneWidth/2);
                rect_dir.y /= len/(laneWidth/2);

                Point2d laneEstimation(currP.x+rect_dir.x,currP.y+rect_dir.y);

                currentStreetEstimate.push_back(laneEstimation);
            }
        }

        //this set was classified to be a left lane
       else if(left_lane_votes >= 0.5*currSet->consensusPoints.size())
       // else if(left_weights >= 0.3 && left_lane_votes > 5)
        {
            for(int i = 0;i < (int)currSet->consensusPoints.size();i++)
            {
                //Point2d currP = *currSet->consensusPoints.at(i);
                Point2d temp = *currSet->consensusPoints.at(i);
                Point2d currP(currSet->a*temp.y*temp.y+currSet->b*temp.y+currSet->c,temp.y);

                double m = 2*currSet->a*currP.y + currSet->b;
                Point2d dirVec(10*m,10);
                Point2d rect_dir(10,-10*m);

                double len = sqrt(100+100*m*m);
                // rect_dir.x /= len/(1.5*laneWidth);
                // rect_dir.y /= len/(1.5*laneWidth);
                rect_dir.x /= len/(1.5*laneWidth);
                rect_dir.y /= len/(1.5*laneWidth);

                Point2d laneEstimation(currP.x+rect_dir.x,currP.y+rect_dir.y);

                currentStreetEstimate.push_back(laneEstimation);
            }
        }
    }
}

//use the old history to map the points
void LaneModel::modelGroupLaneMappingImproved(vector<PointSet>* groups)
{
    vector<PointSet> leftGroups;
    vector<PointSet> middleGroups;
    vector<PointSet> rightGroups;


    vector<double> groupingParams = getGroupingParabola();
    double a = groupingParams.at(0);
    double b = groupingParams.at(1);
    double c = groupingParams.at(2);

    for(int i = 0;i < (int)groups->size();i++)
    {
        PointSet* currSet = &groups->at(i);

        int min_y = 10000;
        int max_y = -10000;
        //find out min y,max y
        for(int j = 0;j < (int)currSet->consensusPoints.size();j++)
        {
            Point2d currPoint = *currSet->consensusPoints.at(j);
            if(currPoint.y > max_y)max_y = currPoint.y;
            if(currPoint.y < min_y)min_y = currPoint.y;
        }

        /*
        int closePoints = 0;

        for(int j = 0;j < (int)currSet->consensusPoints.size();j++)
        {
            Point2d currPoint = *currSet->consensusPoints.at(j);
            if(currPoint.y < min_y + 0.2*(max_y-min_y))closePoints++;
        }*/

        int right_lane_votes = 0;
        int mid_lane_votes = 0;
        int left_lane_votes = 0;

        for(int j = 0;j < (int)currSet->consensusPoints.size();j++)
        {
            //Point2d currPoint = *currSet->consensusPoints.at(j);
            Point2d temp = *currSet->consensusPoints.at(i);
            Point2d currPoint(currSet->a*temp.y*temp.y+currSet->b*temp.y+currSet->c,temp.y);

            if(!modelLocked)
            {
                if(currPoint.y > 100)continue;
            }


            //go from y values from [y-30,y+30] for each point in 5 px steps and check for min distance to parabola model
            double minSquareDistance = 99999999;
            for(int k = currPoint.y-30;k < currPoint.y+30;k += 2)
            {
                double lane_x = a*pow(k,2)+b*k+c;
                double squareDistance = sqrt((lane_x-currPoint.x)*(lane_x-currPoint.x)+(k-currPoint.y)*(k-currPoint.y));
                if(squareDistance < minSquareDistance)minSquareDistance = squareDistance;
            }

            double lane_x = a*currPoint.y*currPoint.y + b*currPoint.y + c;


            if(currPoint.x >= lane_x && (minSquareDistance) < laneWidth)
            {
                if(currPoint.y < min_y+0.2*(max_y-min_y)){right_lane_votes++;}
                right_lane_votes++;
            }

            else if(currPoint.x < lane_x && (minSquareDistance) < laneWidth)
            {
                if(currPoint.y < min_y+0.2*(max_y-min_y)){mid_lane_votes++;}
                mid_lane_votes++;
            }

            else if(currPoint.x < lane_x && (minSquareDistance) >= laneWidth && (minSquareDistance) < 2*laneWidth)
            {
                if(currPoint.y < min_y+0.2*(max_y-min_y)){left_lane_votes++;}
                left_lane_votes++;
            }
        }

        //this set was classified to be a right lane
        if(right_lane_votes >= 0.5*currSet->consensusPoints.size())
        {
            rightGroups.push_back(*currSet);
        }

        //this set was classified to be a mid lane
        else if(mid_lane_votes >= 0.5*currSet->consensusPoints.size())
        {

            for(int i = 0;i < (int)currSet->consensusPoints.size();i++)
            {
                //Point2d currP = *currSet->consensusPoints.at(i);
                Point2d temp = *currSet->consensusPoints.at(i);
                Point2d currP(currSet->a*temp.y*temp.y+currSet->b*temp.y+currSet->c,temp.y);

                //derivative
                double m;
                m = 2*currSet->a*currP.y+currSet->b;

                Point2d dirVec(10*m,10);
                Point2d rect_dir(10,-10*m);

                double len = sqrt(100+100*m*m);
                rect_dir.x /= len/(laneWidth/2);
                rect_dir.y /= len/(laneWidth/2);

                Point2d laneEstimation(currP.x+rect_dir.x,currP.y+rect_dir.y);

                currentStreetEstimate.push_back(laneEstimation);
            }

        }

        //this set was classified to be a left lane
       else if(left_lane_votes >= 0.5*currSet->consensusPoints.size())
        {
            leftGroups.push_back(*currSet);
        }
    }

    bool hasRight = false;
    PointSet* mostLeft;
    double x_min = 10000000;
    //for right lane point sets: get the most left point set
    for(int i = 0;i < (int)rightGroups.size();i++)
    {
        PointSet currSet = rightGroups.at(i);
        Point2d parabolaPoint(currSet.a*50*50+currSet.b*50+currSet.c,50);
        if(parabolaPoint.x < x_min)
        {
            hasRight = true;
            x_min = parabolaPoint.x;
            mostLeft = &rightGroups.at(i);
        }
    }

    bool hasLeft = false;
    PointSet* mostRight;
    double x_max = -10000000;
    //for right lane point sets: get the most left point set
    for(int i = 0;i < (int)leftGroups.size();i++)
    {
        PointSet currSet = leftGroups.at(i);
        Point2d parabolaPoint(currSet.a*50*50+currSet.b*50+currSet.c,50);
        if(parabolaPoint.x > x_max)
        {
            hasLeft = true;
            x_max = parabolaPoint.x;
            mostRight = &leftGroups.at(i);
        }
    }

    // map the right lane
    if(hasRight){
        for(int i = 0;i < (int)mostLeft->consensusPoints.size();i++)
        {
            //Point2d currP = *currSet->consensusPoints.at(i);
            Point2d temp = *(mostLeft->consensusPoints.at(i));
            Point2d currP(mostLeft->a*temp.y*temp.y+mostLeft->b*temp.y+mostLeft->c,temp.y);

            //derivative
            double m;
            m = 2*mostLeft->a*currP.y+mostLeft->b;


            Point2d dirVec(10*m,10);
            Point2d rect_dir(10,-10*m);

            double len = sqrt(100+100*m*m);
            rect_dir.x /= len/(laneWidth/2);
            rect_dir.y /= len/(laneWidth/2);

            Point2d laneEstimation(currP.x-rect_dir.x,currP.y-rect_dir.y);

            currentStreetEstimate.push_back(laneEstimation);
        }
    }

    if(hasLeft)
    {
    //map the left lane
    for(int i = 0;i < (int)mostRight->consensusPoints.size();i++)
    {
        //Point2d currP = *currSet->consensusPoints.at(i);
        Point2d temp = *mostRight->consensusPoints.at(i);
        Point2d currP(mostRight->a*temp.y*temp.y+mostRight->b*temp.y+mostRight->c,temp.y);

        double m = 2*mostRight->a*currP.y + mostRight->b;
        Point2d dirVec(10*m,10);
        Point2d rect_dir(10,-10*m);

        double len = sqrt(100+100*m*m);
        // rect_dir.x /= len/(1.5*laneWidth);
        // rect_dir.y /= len/(1.5*laneWidth);
        rect_dir.x /= len/(1.5*laneWidth);
        rect_dir.y /= len/(1.5*laneWidth);

        Point2d laneEstimation(currP.x+rect_dir.x,currP.y+rect_dir.y);

        currentStreetEstimate.push_back(laneEstimation);
    }
    }



}

void LaneModel::applyCarMovementImproved(double dy,double angle_rad)
{



    dy*=2;

    //erase points behind the car
    double deleteThreshold = -80;

    pthread_mutex_lock(&streetHistoryMutex);

    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);

        //delete the point if it is too far behind the car
        if(currP.y < deleteThreshold)
        {
            streetHistory.erase(streetHistory.begin()+i);
            continue;
        }

        //otherwise rotate and move it

        //rotation around (0,-29):
        double s = sin(angle_rad);
        double c = cos(angle_rad);

        double y_offset = 29;

        streetHistory.at(i).x = streetHistory.at(i).x * c - (streetHistory.at(i).y + y_offset)* s;
        streetHistory.at(i).y = streetHistory.at(i).x * s + (streetHistory.at(i).y + y_offset)* c;
        streetHistory.at(i).y  -= y_offset;

        //translation:
       streetHistory.at(i).y -= dy;
    }





    if(a_hist_grouping.size() < 1)return;

    double aAvg = 0.0,bAvg = 0.0,cAvg = 0.0;
    double yBeginAvg = 0.0,yEndAvg = 0.0;
    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        aAvg += a_hist_grouping.at(i);
        bAvg += b_hist_grouping.at(i);
        cAvg += c_hist_grouping.at(i);

        yBeginAvg += validity_y_begin.at(i);
        yEndAvg += validity_y_end.at(i);
    }

    aAvg /= a_hist_grouping.size();
    bAvg /= a_hist_grouping.size();
    cAvg /= a_hist_grouping.size();
    yBeginAvg /= a_hist_grouping.size();
    yEndAvg /= a_hist_grouping.size();

    //push a new point to the history (if there is no point at this position yet)

    //get largest y in current history
    double largestY = -10000;
    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(currP.y > largestY)largestY = currP.y;
    }

    double new_y = yBeginAvg+(yEndAvg-yBeginAvg)/2;

    for(int i = (int)largestY;i < new_y;i++)
    {
        Point2d toPush(aAvg*i*i+bAvg*i+cAvg,i);
        streetHistory.push_back(toPush);
    }

     pthread_mutex_unlock(&streetHistoryMutex);

    /*
    for(int i = 50;i < new_y;i++)
    {
        double xToPush = aAvg*i*i+bAvg*i+cAvg;
        streetHistory.push_back(Point2d(xToPush,i));
    }
    firstUpdate = false;*/

    /*bool push_it = true;

    if(firstUpdate)
    {
        for(int i = 50;i < new_y;i++)
        {
            double xToPush = aAvg*i*i+bAvg*i+cAvg;
            streetHistory.push_back(Point2d(xToPush,i));
        }
        firstUpdate = false;
    }

    for(int i = 0;i < streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(abs(currP.y - new_y) < 1) push_it = false;
    }
    if(push_it)streetHistory.push_back(Point2d(new_x,new_y));
    if(push_it)streetHistory.push_back(Point2d(new_x,new_y+1));
    if(push_it)streetHistory.push_back(Point2d(new_x,new_y-1));
    if(push_it)streetHistory.push_back(Point2d(new_x-1,new_y));
    if(push_it)streetHistory.push_back(Point2d(new_x,new_y+1));
    if(push_it)streetHistory.push_back(Point2d(new_x+1,new_y-1));*/

     //track the parabolas in x and y



}

void LaneModel::improvedUpdate(vector<vector<Point2d> >* points, bool midLaneFound)
{
    //too many bad fits => reset to straight lane
    if(badFitNumber > 10)
    {
//        cout << "RESET SYSTEM" << endl;
        initialize(true,true);
        return;
    }

    //skip the first n image frames
    int n = 20;
    if(counter < n && rejectFirstFrames)
    {
        //keep the old lane model
        counter++;
        //last frame before re initializaion:
        if(counter == 19)
        {
            initialize(false,false);
        }
        return;
    }

    currentStreetEstimate.clear();
    fitted_a.clear();fitted_b.clear();fitted_c.clear();

    //create a point group for the pre grouped points
    vector<PointSet> pointGroups;
    for(int i = 0;i < (int)points->size();i++)
    {
        PointSet p(&points->at(i));
        if(p.validRansacFit)
        {
             pointGroups.push_back(p);
             fitted_a.push_back(p.a);
             fitted_b.push_back(p.b);
             fitted_c.push_back(p.c);
        }
    }

    //try to create the group lane mapping: middle line information there?
    bool goodMiddleEstimate = false;
    if(midLaneFound && points->size() == 2)
    {
            groupLaneMappingMidFound(&pointGroups);

            //check if the new model makes sense and does not differ too much from the old one
            if(certainty > -1)
            {
                PointSet currentParabolaFit(&currentStreetEstimate);
                if(currentParabolaFit.validRansacFit && a_hist_grouping.size() >= 1)
                {
                    //check how much it differs from the old model

                    double aAvg = 0.0,bAvg = 0.0,cAvg = 0.0;
                    double yBeginAvg = 0.0,yEndAvg = 0.0;
                    for(int i = 0;i < (int)a_hist_grouping.size();i++)
                    {
                        aAvg += a_hist_grouping.at(i);
                        bAvg += b_hist_grouping.at(i);
                        cAvg += c_hist_grouping.at(i);

                        yBeginAvg += validity_y_begin.at(i);
                        yEndAvg += validity_y_end.at(i);
                    }

                    aAvg /= a_hist_grouping.size();
                    bAvg /= a_hist_grouping.size();
                    cAvg /= a_hist_grouping.size();
                    yBeginAvg /= a_hist_grouping.size();
                    yEndAvg /= a_hist_grouping.size();

                    double thresh = 10; //cm difference allowed
                    Point2d oldModelClippingPoint(aAvg*50*50+bAvg*50+cAvg,50);
                    Point2d newModelClippingPoint(currentParabolaFit.a*50*50+currentParabolaFit.b*50+currentParabolaFit.c,50);
                    if(abs(oldModelClippingPoint.x - newModelClippingPoint.x) < thresh && modelLocked){
                        goodMiddleEstimate = true;
                        certainty ++;
                    }
                    else if(!modelLocked)
                    {
                        //cout << "BYPASS" << endl;
                        goodMiddleEstimate = true;
                        certainty++;
                        if(certainty > 10)modelLocked = true;
                        if(certainty == 0 && currentStreetEstimate.size() > 20)
                        {
                            a_hist_grouping.clear();b_hist_grouping.clear();c_hist_grouping.clear();
                        }
                    }
                }
            }
            else
            {
                //goodMiddleEstimate = true;
                certainty = 0;
            }

    }

    if(!goodMiddleEstimate)
    {
        //group the points by the old lane model => now current street estimate vector should be filled
        currentStreetEstimate.clear();
       // certainty--;
        if(certainty < 0)certainty = 0;
        modelGroupLaneMappingImproved(&pointGroups);
    }


    //check if current street estimate is good enough (has a reasonable amount of points)
    if(currentStreetEstimate.size() < 20)
    {
        badFitNumber++;
        //cout << "current street estimate too few points to fit a lane model: " << currentStreetEstimate.size() << endl;
        return;
    }

    PointSet currentParabolaFit(&currentStreetEstimate);
    if(!currentParabolaFit.validRansacFit){cout << "INVALID FIT" << endl; return;}

    //find min max y value of the consensus set
    double yBegin = 1000000.0,yEnd = -10000000.0;
    for(int i = 0;i < (int)currentParabolaFit.consensusPoints.size();i++)
    {
        Point2d currP = *currentParabolaFit.consensusPoints.at(i);
        if(currP.y < yBegin)    yBegin = currP.y;
        if(currP.y > yEnd)      yEnd   = currP.y;
    }

    pthread_mutex_lock(&streetHistoryMutex);


    //safe the result
    a_hist_grouping.push_back(currentParabolaFit.a);
    b_hist_grouping.push_back(currentParabolaFit.b);
    c_hist_grouping.push_back(currentParabolaFit.c);

    validity_y_begin.push_back(yBegin);
    validity_y_end.push_back(yEnd);

    if(a_hist_grouping.size() == 5)
    {
        a_hist_grouping.erase(a_hist_grouping.begin());
        b_hist_grouping.erase(b_hist_grouping.begin());
        c_hist_grouping.erase(c_hist_grouping.begin());

        validity_y_begin.erase(validity_y_begin.begin());
        validity_y_end.erase(validity_y_end.begin());
    }

    pthread_mutex_unlock(&streetHistoryMutex);




    //fit the model to the history:

    /*
    vector<Point2d> controlPointsGrouping;
    //Point2d controlPoint4(a*50*50+b*50+c,50);
    Point2d controlPoint(currentParabolaFit.a*50*50+currentParabolaFit.b*50+currentParabolaFit.c,50);
    controlPointsGrouping.push_back(controlPoint);


    PointSet historySet(&streetHistory,controlPointsGrouping);
    if(historySet.validRansacFit)
    {
        a_hist.push_back(historySet.a);
        b_hist.push_back(historySet.b);
        c_hist.push_back(historySet.c);
    }*/

    //fit the history parabola, constraining one point

    pthread_mutex_lock(&streetHistoryMutex);

    if(a_hist_grouping.size() < 1)
    {
        pthread_mutex_unlock(&streetHistoryMutex);
        return;
    }

    double aAvg = 0.0,bAvg = 0.0,cAvg = 0.0;
    double yBeginAvg = 0.0,yEndAvg = 0.0;

    int history_usage_size = MIN(USE_HISTORY_SIZE,a_hist_grouping.size());


    /*for(int i = 0;i < a_hist_grouping.size();i++)
    {
        aAvg += a_hist_grouping.at(i);
        bAvg += b_hist_grouping.at(i);
        cAvg += c_hist_grouping.at(i);

        yBeginAvg += validity_y_begin.at(i);
        yEndAvg += validity_y_end.at(i);
    }*/

    for(int i = 0;i < history_usage_size;i++)
    {
        aAvg += a_hist_grouping.at(a_hist_grouping.size()-1-i);
        bAvg += b_hist_grouping.at(a_hist_grouping.size()-1-i);
        cAvg += c_hist_grouping.at(a_hist_grouping.size()-1-i);

        yBeginAvg += validity_y_begin.at(a_hist_grouping.size()-1-i);
        yEndAvg += validity_y_end.at(a_hist_grouping.size()-1-i);
    }

    aAvg /= history_usage_size;
    bAvg /= history_usage_size;
    cAvg /= history_usage_size;
    yBeginAvg /= history_usage_size;
    yEndAvg /= history_usage_size;

    pthread_mutex_unlock(&streetHistoryMutex);

/*  Only take the last sampled lane model for fitting the history
    aAvg = a_hist_grouping.at(a_hist_grouping.size()-1);
    bAvg = b_hist_grouping.at(a_hist_grouping.size()-1);
    cAvg = c_hist_grouping.at(a_hist_grouping.size()-1);
    yBeginAvg = validity_y_begin.at(validity_y_begin.size()-1);
    yEndAvg   = validity_y_end.at(validity_y_end.size()-1);
    */


    double yConnection = 50;//was 50
    Point2d p1(aAvg*yConnection*yConnection +bAvg*yConnection + cAvg,yConnection);

    //double m = 2*aAvg*yBeginAvg+bAvg;
   // Point2d p2 = p1 + Point2d(2*m,2);

    int ransacIterations = 50;
    double error_margin = 2;
    double bestConsensusValue = 0;

    bool fitFound = false;
    double best_a,best_b,best_c;

    //pthread_mutex_lock(&streetHistoryMutex);

    vector<Point2d> pointsToFit = streetHistory;
    /*for(int i = 0;i < streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(currP.y < 50)pointsToFit.push_back(currP);
    }*/

    //pthread_mutex_unlock(&streetHistoryMutex);

    for(int i = 0;i < ransacIterations;i++)
    {
        //sample two random points from the history
       // Point2d p1 = pointsToFit.at(rand.uniform(0, pointsToFit.size()));
        Point2d p2 = pointsToFit.at(rand.uniform(0, pointsToFit.size()));



        Point2d p3 = pointsToFit.at(rand.uniform(0, pointsToFit.size()));

        //solve parabola parameters
        Mat A = (Mat_<double>(3,3) <<   p1.y*p1.y,p1.y,1,
                                        p2.y*p2.y,p2.y,1,
                                        p3.y*p3.y,p3.y,1);

        Mat b = (Mat_<double>(3,1) <<   p1.x,p2.x,p3.x);

        Mat sol;

        solve(A, b, sol);

        double aFit = sol.at<double>(0,0);
        double bFit = sol.at<double>(1,0);
        double cFit = sol.at<double>(2,0);



        int consensusValue = 0;

        for(int i = 0;i < (int)pointsToFit.size();i++)
        {
            double y = (pointsToFit.at(i)).y;
            double x = aFit*y*y + bFit*y+ cFit;
            double error = abs(x-(pointsToFit.at(i).x));
            if(error < error_margin)
            {
                consensusValue += 1;
            }
            else
            {
                consensusValue += 0;
            }
        }

        if(consensusValue > bestConsensusValue)
        {
            fitFound = true;
            bestConsensusValue = consensusValue;
            best_a = aFit;
            best_b = bFit;
            best_c = cFit;
        }
    }

    //check with old model if reliable

    double aAvg2 = 0.0,bAvg2 = 0.0,cAvg2 = 0.0;


    /*for(int i = 0;i < a_hist_grouping.size();i++)
    {
        aAvg += a_hist_grouping.at(i);
        bAvg += b_hist_grouping.at(i);
        cAvg += c_hist_grouping.at(i);

        yBeginAvg += validity_y_begin.at(i);
        yEndAvg += validity_y_end.at(i);
    }*/

    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        aAvg2 += a_hist_grouping.at(i);
        bAvg2 += b_hist_grouping.at(i);
        cAvg2 += c_hist_grouping.at(i);
    }

    aAvg2 /= a_hist_grouping.size();
    bAvg2 /= a_hist_grouping.size();
    cAvg2 /= a_hist_grouping.size();

    double thresh = 10; //cm difference allowed
    Point2d oldModelClippingPoint(aAvg2*50*50+bAvg2*50+cAvg2,50);
    Point2d newModelClippingPoint(best_a*50*50+best_b*50+best_c,50);
    //cout << "DIST: " << abs(oldModelClippingPoint.x - newModelClippingPoint.x) << endl;
    if(abs(oldModelClippingPoint.x - newModelClippingPoint.x) > thresh && modelLocked){
        badFitNumber++;

        if(badFitNumber > 3)
        {
            initialize(true,true);
            cout << "INIT" << endl;
        }
        return;
    }
    else
    {
        badFitNumber--;
        if(badFitNumber < 0)badFitNumber = 0;
    }

    if(fitFound)
    {
        a_hist.push_back(best_a);
        b_hist.push_back(best_b);
        c_hist.push_back(best_c);


        if(a_hist.size() == USE_HISTORY_SIZE)
        {
            a_hist.erase(a_hist.begin());
            b_hist.erase(b_hist.begin());
            c_hist.erase(c_hist.begin());
        }
    }
}


void LaneModel::preGroupedUpdate(vector<vector<Point2d> >* points)
{

    //ignore the first n frames until the camerea is ready
    int n = 20;
    if(counter < n)
    {
        counter++;
        return;
    }

    currentStreetEstimate.clear();

    //erase the oldest history entries
    if(a_hist.size() == 2)
    {
        a_hist.erase (a_hist.begin());
        b_hist.erase (b_hist.begin());
        c_hist.erase (c_hist.begin());
    }

    if(a_hist_grouping.size() == 2)
    {
        a_hist_grouping.erase(a_hist_grouping.begin());
        b_hist_grouping.erase(b_hist_grouping.begin());
        c_hist_grouping.erase(c_hist_grouping.begin());
    }


    //create a point group for the pre grouped points
    vector<PointSet> pointGroups;
    for(int i = 0;i < (int)points->size();i++)
    {
        PointSet p(&points->at(i));
        if(p.validRansacFit)
             pointGroups.push_back(p);
    }


    //group the points by the old lane model => now left/mid/rightPoints vectors should be filled
    modelGroupLaneMapping(&pointGroups);


    //update the parabola estimate
    estimateLane();


    updateHistory();
}


void LaneModel::updateHistory()
{
    pthread_mutex_lock(&streetHistoryMutex);
    //delete points that are too far behind the car
    int delete_thresh = -100;

    //clean history => delete points that are too far behind the car
    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(currP.y < delete_thresh)
        {
            streetHistory.erase(streetHistory.begin()+i);
            i--;
        }
    }

    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        a += a_hist_grouping.at(i);
        b += b_hist_grouping.at(i);
        c += c_hist_grouping.at(i);
    }

    a /= a_hist_grouping.size();
    b /= a_hist_grouping.size();
    c /= a_hist_grouping.size();



    //delete the history points super far away from the current model
    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currPoint = streetHistory.at(i);

        if(currPoint.y < 50)continue;


        //go from y values from [y-30,y+30] for each point in 5 px steps and check for min distance to parabola model
        double minSquareDistance = 99999999;
        for(int k = currPoint.y-30;k < currPoint.y+30;k += 2)
        {
            double lane_x = a*pow(k,2)+b*k+c;
            double squareDistance = sqrt((lane_x-currPoint.x)*(lane_x-currPoint.x)+(k-currPoint.y)*(k-currPoint.y));
            if(squareDistance < minSquareDistance)minSquareDistance = squareDistance;
        }

        if(minSquareDistance > 20)streetHistory.erase(streetHistory.begin()+i);

    }


    //delete duplicate points in the street history
    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        int similarPoints = 0;

        for(int j = i+1;j < (int)streetHistory.size();j++)
        {
            Point2d compP = streetHistory.at(j);

            Point2d diffP = compP-currP;
            double dist = sqrt(diffP.x*diffP.x+diffP.y*diffP.y);

            if(dist < 2)
            {
                similarPoints++;
                if(similarPoints > 4)streetHistory.erase(streetHistory.begin()+j);
            }
        }
    }


    //clear -100 to 0

    /*

    vector<Point2d> backPoints,midPoints,frontPoints,frontPoints2;
    for(int i = 0;i < streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(currP.y < 0)backPoints.push_back(currP);
        else if(currP.y >= 0 && currP.y < 50)midPoints.push_back(currP);
        else if(currP.y >= 50 && currP.y < 100)frontPoints.push_back(currP);
        else frontPoints2.push_back(currP);
    }

    PointSet backSet(&backPoints);
    PointSet midSet(&midPoints);
    PointSet frontSet(&frontPoints);
    PointSet frontSet2(&frontPoints2);

    streetHistory.clear();

    if(backPoints.size() > 100 && backSet.validRansacFit && backSet.consensusPoints.size() > 0.8*backSet.points.size())
    {
        for(int i = 0;i < backSet.consensusPoints.size();i++)
        {
            Point2d toPush = *backSet.consensusPoints.at(i);
            streetHistory.push_back(toPush);
        }
    }
    else
    {
        streetHistory.insert(streetHistory.end(),backPoints.begin(),backPoints.end());
    }

    if(midPoints.size() > 100 && midSet.validRansacFit && midSet.consensusPoints.size() > 0.8*midSet.points.size())
    {
        for(int i = 0;i < midSet.consensusPoints.size();i++)
        {
            Point2d toPush = *midSet.consensusPoints.at(i);
            streetHistory.push_back(toPush);
        }
    }
    else
    {
        streetHistory.insert(streetHistory.end(),midPoints.begin(),midPoints.end());
    }

    if(frontSet.validRansacFit && frontSet.consensusPoints.size() > 0.8*frontSet.points.size())
    {
        for(int i = 0;i < frontSet.consensusPoints.size();i++)
        {
         Point2d toPush = *frontSet.consensusPoints.at(i);
            streetHistory.push_back(toPush);
        }
    }
    else
    {
        streetHistory.insert(streetHistory.end(),frontPoints.begin(),frontPoints.end());
    }

    if(frontSet2.validRansacFit && frontSet2.consensusPoints.size() > 0.8*frontSet2.points.size())
    {
        for(int i = 0;i < frontSet2.consensusPoints.size();i++)
        {
         Point2d toPush = *frontSet2.consensusPoints.at(i);
            streetHistory.push_back(toPush);
        }
    }
    else
    {
        streetHistory.insert(streetHistory.end(),frontPoints2.begin(),frontPoints2.end());
    }




*/


    pthread_mutex_unlock(&streetHistoryMutex);
}


//distance in cm - points in the history must be estimated
void LaneModel::applyCarMovement(double dy,double angle_rad)
{

    if(abs(dy)>100.0 || abs(angle_rad)>0.7) {
        return;
    }

    yAcc += dy;
    angAcc += angle_rad;

    if(abs(yAcc) < 2)return;

    angle_rad = angAcc;
    dy = yAcc;
    yAcc = 0.0;
    angAcc = 0.0;




    pthread_mutex_lock(&streetHistoryMutex);

    //rotate all the points in the history by angle_rad around the car at (0,0)

   double s = sin(angle_rad);
    double c = cos(angle_rad);

    double y_offset = 29;

    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        streetHistory.at(i).x = streetHistory.at(i).x * c - (streetHistory.at(i).y + y_offset)* s;
        streetHistory.at(i).y = streetHistory.at(i).x * s + (streetHistory.at(i).y + y_offset)* c;
        streetHistory.at(i).y  -= y_offset;
    }


    //Move the car by dy
    //drivenDistance += dy;
    //xShift += sin(angle_rad)*dy;


    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        if(i < 0 || i >= (int)streetHistory.size()){cout << "ERROR: 252 in LaneModel: i: " << i << " size: " << streetHistory.size() << endl;}
        streetHistory.at(i).y -= dy;
    }

    pthread_mutex_unlock(&streetHistoryMutex);
}


void LaneModel::estimateLane()
{

    pthread_mutex_lock(&streetHistoryMutex);

    //take the lane estimate from all currently the given points and do a ransac fit
    PointSet currentStreetSet(&currentStreetEstimate);

    if(currentStreetSet.validRansacFit){
        //push those good points to the history and fit the history again for a final estimate
        for(int i = 0;i < (int)currentStreetSet.consensusPoints.size();i++)
        {
            double y = currentStreetSet.consensusPoints.at(i)->y;




            //delete all the occurences of this y from the history
            for(int j = 0;j < (int)streetHistory.size();j++)
            {
                if(abs(streetHistory.at(j).y - y) <= 2)
                {
                    streetHistory.erase(streetHistory.begin()+j);
                    j--;
                }
            }

        }
        for(int i = 0;i < (int)currentStreetSet.consensusPoints.size();i++)
        {
            Point2d newPoint;
            newPoint.x = (*currentStreetSet.consensusPoints.at(i)).x;
            newPoint.y = (*currentStreetSet.consensusPoints.at(i)).y;
            streetHistory.push_back(newPoint);
        }
    }



    //remove all the non consensus points from the street history
    /*
    vector<Point2d> historyConsensusPoints;
    PointSet streetHistoryFit(&streetHistory);
    for(int i = 0;i < streetHistoryFit.consensusPoints.size();i++)
    {
        historyConsensusPoints.push_back(*streetHistoryFit.consensusPoints.at(i));
    }
    streetHistory = historyConsensusPoints;

    */

    //now fit the resulting road model in the range


    //get current parabola parameters for lane model
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        a += a_hist.at(i);
        b += b_hist.at(i);
        c += c_hist.at(i);
    }

    a /= a_hist.size();
    b /= b_hist.size();
    c /= c_hist.size();

    //get current parabola parameters for grouping

    double a_group = 0.0;
    double b_group = 0.0;
    double c_group = 0.0;

    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        a_group += a_hist_grouping.at(i);
        b_group += b_hist_grouping.at(i);
        c_group += c_hist_grouping.at(i);
    }

    a_group /= a_hist_grouping.size();
    b_group /= a_hist_grouping.size();
    c_group /= a_hist_grouping.size();

    //fit new lane model with constraints

    vector<Point2d> pointsToFit;

    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(currP.y > -90 && currP.y < 60)
        {
            pointsToFit.push_back(currP);
        }
    }


    vector<Point2d> controlPointsLane;
    Point2d controlPoint1(c,0);
    Point2d controlPoint2(a*50*50+b*50+c,50);
    //Point2d controlPoint3(a_group*50*50+b_group*50+c_group,50);
    controlPointsLane.push_back(controlPoint1);
    controlPointsLane.push_back(controlPoint2);
    //controlPointsLane.push_back(controlPoint3);
    PointSet finalRoadEstimate(&pointsToFit,controlPointsLane);

    if(finalRoadEstimate.validRansacFit)
    {
        a_hist.push_back(finalRoadEstimate.a);
        b_hist.push_back(finalRoadEstimate.b);
        c_hist.push_back(finalRoadEstimate.c);
    }


    //update the grouping parameters by using the history in the camera region


    pointsToFit.clear();

    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        if(currP.y > 50 && currP.y < 150)
        {
            pointsToFit.push_back(currP);
        }
    }




    vector<Point2d> controlPointsGrouping;
    //Point2d controlPoint4(a*50*50+b*50+c,50);
    Point2d controlPoint5(a_group*50*50+b_group*50+c_group,50);
    Point2d controlPoint6(a_group*80*80+b_group*80+c_group,80);
   // controlPointsGrouping.push_back(controlPoint4);
    controlPointsGrouping.push_back(controlPoint5);
    controlPointsGrouping.push_back(controlPoint6);

    PointSet groupingEstimate(&pointsToFit,controlPointsGrouping);

    if(groupingEstimate.validRansacFit)
    {
        a_hist_grouping.push_back(groupingEstimate.a);
        b_hist_grouping.push_back(groupingEstimate.b);
        c_hist_grouping.push_back(groupingEstimate.c);
    }


    pthread_mutex_unlock(&streetHistoryMutex);
}

int LaneModel::getCarState(double* curvature,double* distanceRightLane,double* angle,bool* isCurve)
{
    if(a_hist.size() <= 0 || b_hist.size() <= 0 || c_hist.size() <= 0 || a_hist_grouping.size() <= 0 || b_hist_grouping.size() <= 0 || c_hist_grouping.size() <= 0)
    {
        return -1;
    }

    //calculate model parameters by averaging
    double a = 0;
    double b = 0;
    double c = 0;



    for(size_t i = 0;i < a_hist.size();i++)
    {
        a+= a_hist.at(i);
        b+= b_hist.at(i);
        c+= c_hist.at(i);
    }

    a/= a_hist.size();
    b/= b_hist.size();
    c/= c_hist.size();


    pthread_mutex_lock(&streetHistoryMutex);

    double aGroup = 0,bGroup = 0,cGroup = 0;
    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        aGroup += a_hist_grouping.at(i);
        bGroup += b_hist_grouping.at(i);
        cGroup += c_hist_grouping.at(i);
    }
    aGroup /= a_hist_grouping.size();
    bGroup /= b_hist_grouping.size();
    cGroup /= c_hist_grouping.size();

    pthread_mutex_unlock(&streetHistoryMutex);

    double dist1 = laneWidth/2.0+c;
    double dist2 = laneWidth/2.0+a*(-30)*(-30)+b*(-30)+c;
    double dist = (dist1+dist2)/2.0;

    Point2d carP1(a*(-30)*(-30)+b*(-30)+c,-30);
    Point2d carP2(c,0);
    Point2d tangentVec = carP2-carP1;

    double tan_angle = tangentVec.x / tangentVec.y;
    double ang = -atan(tan_angle);


    //get the curvature by fitting a circle to 3 points on the parabola model
    /*
    int curv_y_1 = 0;
    int curv_y_2 = 20;
    int curv_y_3 = 40;


    Point2d circP1(a*curv_y_1*curv_y_1+b*curv_y_1+c,curv_y_1);
    Point2d circP2(a*curv_y_2*curv_y_2+b*curv_y_2+c,curv_y_2);
    Point2d circP3(aGroup*curv_y_3*curv_y_3+bGroup*curv_y_3+cGroup,curv_y_3);


    Mat A = (Mat_<double>(3,3) <<   1,-circP1.x,-circP1.y,
             1,-circP2.x,-circP2.y,
             1,-circP3.x,-circP3.y);

    Mat b_vec = (Mat_<double>(3,1) << -(circP1.x*circP1.x+circP1.y*circP1.y),
                 -(circP2.x*circP2.x+circP2.y*circP2.y),
                 -(circP3.x*circP3.x+circP3.y*circP3.y));

    Mat sol;
    solve(A,b_vec,sol);

    double a_param = sol.at<double>(0,0);
    double b_param = sol.at<double>(1,0);
    double c_param = sol.at<double>(2,0);

    double circ_x = b_param/2;
    double circ_y = c_param/2;
    double circ_r = sqrt(circ_x*circ_x+circ_y*circ_y-a_param);*/

/*
    Circle circ(circP1,circP2,circP3);

    double curve = 0;
    if(circ.isValid)
    {
        curve = 1.0/circ.radius;
        if(circ.center.x > 0)curve*=-1;
        *curvature = curve;
    }
    else
    {
        *curvature = 0.0;
    }*/

    /*
    double curvatureOutput = 1.0/circ_r;
    if(circ_x > 0)curvatureOutput*=-1;
    if(abs(circ_r) != abs(circ_r))*curvature = 0.0;
    else if(abs(circ_r) < 50)*curvature = 0.0;
    else if(abs(circ_r) > 1000)*curvature = 0.0;
    else *curvature = curvatureOutput;*/


   /* if(abs(circ_r) != abs(circ_r)) *curvature = 0.0;
    else if(abs(circ_r) > 3000)   *curvature = 0.0;
    else if(abs(circ_r) < 20)   *curvature = 0.0;
    else *curvature = 1/(circ_r);*/

    //get curvature by paraobola formula

    double curvatureTemp = 0.0;
    for(int i = 0;i < 50;i++)
    {
        curvatureTemp += (2*a)/pow((1+pow(a*i+b,2.0)),1.5);
    }
    *curvature = curvatureTemp / 50.0;

    
    double curvature_top = 0.0;
    for(int i = 50;i < 150;i++)
    {
        curvature_top += (2*aGroup)/pow((1+pow(aGroup*i+bGroup,2.0)),1.5);
    }
    curvature_top /= 100.0;


    if(abs(curvature_top) > 1/400.0)
    {
        *isCurve = true;
        isCurveCounter = 10;
    }
    else
    {
        if(isCurveCounter > 0)
        {
            *isCurve = true;
            isCurveCounter--;
        }
        else
        {
            *isCurve = false;
        }
    }



    *distanceRightLane = dist;
    *angle = ang;

    return 0;
}

vector<double> LaneModel::getGroupingParabola()
{
    double a_group = 0;
    double b_group = 0;
    double c_group = 0;

    for(int i = 0;i < (int)a_hist_grouping.size();i++)
    {
        a_group += a_hist_grouping.at(i);
        b_group += b_hist_grouping.at(i);
        c_group += c_hist_grouping.at(i);
    }

    a_group /= a_hist_grouping.size();
    b_group /= b_hist_grouping.size();
    c_group /= c_hist_grouping.size();

    vector<double> result;
    result.push_back(a_group);
    result.push_back(b_group);
    result.push_back(c_group);
    return result;
}

vector<double> LaneModel::getHistoryParabola()
{
    double a = 0;
    double b = 0;
    double c = 0;

    for(size_t i = 0;i < a_hist.size();i++)
    {
        a+= a_hist.at(i);
        b+= b_hist.at(i);
        c+= c_hist.at(i);
    }

    a/= a_hist.size();
    b/= a_hist.size();
    c/= a_hist.size();

    vector<double> result;
    result.push_back(a);
    result.push_back(b);
    result.push_back(c);
    return result;
}

void LaneModel::getDebugImage(Mat laneModelDrawing)
{
    vector<double> histParams = getHistoryParabola();
    vector<double> groupParams = getGroupingParabola();
    double a = histParams.at(0);
    double b = histParams.at(1);
    double c = histParams.at(2);

    double a_group = groupParams.at(0);
    double b_group = groupParams.at(1);
    double c_group = groupParams.at(2);

    //---------------------- DEBUG OUTPUT LANE MODEL---------------------------------//
    int carOffset = 50;

    //draw the street history points - red line
    for(int i = 0;i < (int)streetHistory.size();i++)
    {
        Point2d currP = streetHistory.at(i);
        currP.x += image_w_half;
        currP.y += carOffset;
        circle(laneModelDrawing,currP,1,Scalar(0,0,255),-1);        
    }

    //draw the parabola in section -50 to 50
    for(int y = -50;y < 50;y++)
        //draw the parabola model - blue line
    {
        double x1 = a*y*y + b*y+c;
        double x2 = a*(y+1)*(y+1) + b*(y+1)+c;
        line(laneModelDrawing,Point(x1+image_w_half,y+50),Point(x2+image_w_half,y+51),Scalar(255,0,0),1);
    }

    //draw the parabola model in section 50 to 160
    for(int y = 50;y < 160;y++)
    {
        double x1g = a_group*y*y + b_group*y+c_group;
        double x2g = a_group*(y+1)*(y+1) + b_group*(y+1)+c_group;
        line(laneModelDrawing,Point(x1g+image_w_half,y+50),Point(x2g+image_w_half,y+51),Scalar(0,255,0),1);//not shown?

        //paint the fitted parabolas throgh the point sets:
        for(int j = 0;j < (int)fitted_a.size();j++)
        {
            double x1g = fitted_a.at(j)*y*y + fitted_b.at(j)*y+fitted_c.at(j);
            double x2g = fitted_a.at(j)*(y+1)*(y+1) + fitted_b.at(j)*(y+1)+fitted_c.at(j);
            line(laneModelDrawing,Point(x1g+image_w_half,y+50),Point(x2g+image_w_half,y+51),Scalar(0,255,255),1);//not shown?
        }
    }

    //draw the cars location
    circle(laneModelDrawing,Point(image_w_half,50),3,Scalar(255,255,255),-1);

    //print model certainty
    std::ostringstream strs;
    strs << "Certainty: " << certainty;
    std::string str = strs.str();
    //putText(laneModelDrawing,str,Point(10,25),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);

    //draw the current street estimate - green line
    for(int i = 0;i < (int)currentStreetEstimate.size();i++)
    {
        circle(laneModelDrawing,currentStreetEstimate.at(i)+Point2d(image_w_half,50),1,Scalar(0,255,0),-1);
    }
}
