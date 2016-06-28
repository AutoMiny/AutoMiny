//
//  PointSet.cpp
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

#include "PointSet.h"

PointSet::PointSet(vector<Point2d>* points_arg)
{
    a = b = c = 0;

    //convert the input points
    vector<Point2d*> converted_arg;
    for(size_t i = 0;i < points_arg->size();i++)
    {
        converted_arg.push_back(&points_arg->at(i));
    }

    points = converted_arg;

    vector<Point2d> empty_vec;
    fitRansacParabola(empty_vec);
}

PointSet::PointSet(vector<Point2d>* points_arg,vector<Point2d> controlPoints)
{
    a = b = c = 0;

    //convert the input points
    vector<Point2d*> converted_arg;
    for(size_t i = 0;i < points_arg->size();i++)
    {
        converted_arg.push_back(&points_arg->at(i));
    }

    points = converted_arg;

    fitRansacParabola(controlPoints);
}



PointSet::PointSet(vector<Point2d*>* points_arg)
{
    a = b = c = 0;

    for(size_t i = 0;i < points_arg->size();i++)
        points.push_back(points_arg->at(i));

    vector<Point2d> empty_vec;
    fitRansacParabola(empty_vec);
}



//solves the parabola equation parametrized by the y-values of the 3 points
void PointSet::getParabola(Point2d p1,Point2d p2,Point2d p3,vector<double>* result)
{
    Mat A = (Mat_<double>(3,3) <<   p1.y*p1.y,p1.y,1,
                                    p2.y*p2.y,p2.y,1,
                                    p3.y*p3.y,p3.y,1);

    Mat b = (Mat_<double>(3,1) <<   p1.x,p2.x,p3.x);

    Mat sol;

    solve(A, b, sol);

    //assert size is 3
    if(sol.rows != 3)cout << "ERROR IN GET PARABOLA: SIZE IS : " << sol.size() << endl;

    result->push_back(sol.at<double>(0,0));
    result->push_back(sol.at<double>(1,0));
    result->push_back(sol.at<double>(2,0));
}

void PointSet::getLine(Point2d p1,Point2d p2,vector<double>* result)
{
    if(p1 == p2){result->push_back(0.0);result->push_back(0.0);result->push_back(0.0);}

    Point2d diff = p2-p1;
    double m = diff.x / diff.y;
    double t = p1.x-m*p1.y;

    result->push_back(0.0);
    result->push_back(m);
    result->push_back(t);
}

void PointSet::fitRansacParabola(vector<Point2d> controlPoints)
{
    validRansacFit = false;
    int totalPoints = points.size();
    if(totalPoints < 10){
        cout << "NO POINTS GiVEN" << endl;
        return;
    }


    double minErrorParabola = 99999999999;
    double minErrorLine     = 99999999999;
    int iterations = 0;
    int error_margin = 3;

    vector<double> bestParamsParabola;
    vector<double> bestParamsLine;
    vector<Point2d*> bestConsensusSetParabola;
    vector<Point2d*> bestConsensusSetLine;

    while(iterations < 50)
    {
        vector<Point2d*> parabolaConsensusSet;
        vector<Point2d*> lineConsensusSet;
        iterations++;

        Point2d p1,p2,p3;
        do{
            p1 = *points.at(rand.uniform(0, points.size()));
            p2 = *points.at(rand.uniform(0, points.size()));
            p3 = *points.at(rand.uniform(0, points.size()));
        }while(p1 == p2 || p1 == p3 || p2 == p3);

        //if(controlPoints.size() > 0)p3 = controlPoints.at(0);

        vector<double> resultParams;
        vector<double> resultParamsLine;

        //fit line and parabola
        getLine(p1,p2,&resultParamsLine);
        getParabola(p1, p2, p3, &resultParams);

        //compute error for line and parabola
        double parabolaError = 0.0;
        double lineError = 0.0;

        for(int i = 0;i < (int)points.size();i++)
        {
            double y = (*points.at(i)).y;
            double x_parabola = resultParams.at(0)*y*y + resultParams.at(1)*y+ resultParams.at(2);
            double x_line     = resultParamsLine.at(1)*y + resultParamsLine.at(2);
            double error_parabola = abs(x_parabola-(*points.at(i)).x);
            double error_line = abs(x_line-(*points.at(i)).x);

            if(error_parabola < error_margin)
            {
                parabolaError += error_parabola;
                parabolaConsensusSet.push_back(points.at(i));
            }
            else
            {
                parabolaError += 2*error_margin;
            }

            if(error_line < error_margin)
            {
                lineError += error_line;
                lineConsensusSet.push_back(points.at(i));
            }
            else
            {
                lineError += 2*error_margin;
            }
        }


        if(parabolaError < minErrorParabola && abs(resultParams.at(0)) < 5)
        {
            minErrorParabola = parabolaError;
            bestParamsParabola = resultParams;
            bestConsensusSetParabola = parabolaConsensusSet;
        }
        if(lineError < minErrorLine)
        {
            minErrorLine = lineError;
            bestParamsLine = resultParamsLine;
            bestConsensusSetLine = lineConsensusSet;
        }

/*
        bool controlPointsOK = true;
        for(int i = 0;i < controlPoints.size();i++)
        {
            Point2d currPoint = controlPoints.at(i);
            double y = currPoint.y;
            double x = resultParams.at(0)*y*y + resultParams.at(1)*y+ resultParams.at(2);
            if(abs(x-currPoint.x) > 5)controlPointsOK = false;
        }
        if(!controlPointsOK)continue;

        //evaluate the fit

*/

    }

    //check which fit was better:
    if(minErrorParabola < 0.75*minErrorLine)
    {
        //parabola better than line:
        if(bestParamsParabola.size() < 3){cout << "NO SUCCESS FITTING" << endl;return;}
        validRansacFit = (bestConsensusSetParabola.size() > 10);
        a = bestParamsParabola.at(0);
        b = bestParamsParabola.at(1);
        c = bestParamsParabola.at(2);
        consensusPoints = bestConsensusSetLine;
    }
    else
    {
        //parabola better than line:
        if(bestParamsLine.size() < 3){cout << "NO SUCCESS FITTING" << endl;return;}
        validRansacFit = (bestConsensusSetLine.size() > 10);
        a = 0.0;
        b = bestParamsLine.at(1);
        c = bestParamsLine.at(2);
        consensusPoints = bestConsensusSetLine;
    }

}

//----FUNCTIONS THAT ARE NOT USED AT THE MOMENT AND ARE NEVER CALLED-----//

//solves the cubic spline equation parametrized by the y-values of the 4 points
void PointSet::getCubicSpline(Point2d p1,Point2d p2,Point2d p3,Point2d p4,vector<double>* result)
{
    //-------------------------DEPRECATED WARNING--------------------------------//
    cout << "FUNCTION CALL TO GET CUBIC SPLINE in PointSet - SHOULD NOT BE USED !" << endl;
    return;
    //-------------------------DEPRECATED WARNING--------------------------------//


    double y1_1 = p1.y; double y1_2 = y1_1*p1.y; double y1_3 = y1_2*p1.y;
    double y2_1 = p2.y; double y2_2 = y2_1*p2.y; double y2_3 = y2_2*p2.y;
    double y3_1 = p3.y; double y3_2 = y3_1*p3.y; double y3_3 = y3_2*p3.y;
    double y4_1 = p4.y; double y4_2 = y4_1*p4.y; double y4_3 = y4_2*p4.y;

    Mat A = (Mat_<double>(4,4) <<   y1_3,y1_2,y1_1,1,
                                    y2_3,y2_2,y2_1,1,
                                    y3_3,y3_2,y3_1,1,
                                    y4_3,y4_2,y4_1,1);

    Mat b = (Mat_<double>(4,1) <<   p1.x,p2.x,p3.x,p4.x);

    Mat sol;

    solve(A, b, sol);

    result->push_back(sol.at<double>(0,0));
    result->push_back(sol.at<double>(1,0));
    result->push_back(sol.at<double>(2,0));
    result->push_back(sol.at<double>(3,0));
}




void PointSet::fitLinear()
{
    //-------------------------DEPRECATED WARNING--------------------------------//
    cout << "FUNCTION CALL TO GET LINEAR in PointSet - SHOULD NOT BE USED !" << endl;
    return;
    //-------------------------DEPRECATED WARNING--------------------------------//

    double n = points.size();
    double x_avg = 0.0;
    double y_avg = 0.0;
    double sum_x_2   = 0.0;
    double sum_x_y = 0.0;
    double sum_y_2 = 0.0;

    for(size_t i = 0;i < n;i++)
    {
        //note: we swap x/y of the input points to make a better fit
        double px = points.at(i)->y;
        double py = points.at(i)->x;
        x_avg += px;
        y_avg += py;
        sum_x_2 += (px*px);
        sum_x_y += (px*py);
        sum_y_2 += (py*py);
    }
    x_avg /= n;
    y_avg /= n;

    double m = (sum_x_y - n*x_avg*y_avg) / (sum_x_2-n*x_avg*x_avg);
    double t = y_avg-m*x_avg;

    a = 0;
    b = m;
    c = t;
}




void PointSet::fitParabola()
{

    //-------------------------DEPRECATED WARNING--------------------------------//
    cout << "FUNCTION CALL TO FIT PARABOLA in PointSet - SHOULD NOT BE USED !" << endl;
    return;
    //-------------------------DEPRECATED WARNING--------------------------------//


    double n = points.size();
    double sum_xi_1  = 0.0;
    double sum_xi_2  = 0.0;
    double sum_xi_3  = 0.0;
    double sum_xi_4  = 0.0;

    double sum_right1   = 0.0;
    double sum_right2   = 0.0;
    double sum_right3   = 0.0;


    for(size_t i = 0;i < n;i++)
    {
        //note: we swap x/y of the input points to make a better fit
        double px = points.at(i)->y;
        double py = points.at(i)->x;

        sum_xi_1     += px;
        sum_xi_2     += (px*px);
        sum_xi_3     += (px*px*px);
        sum_xi_4     += (px*px*px*px);


        sum_right1   += px*px*py;
        sum_right2   += px*py;
        sum_right3   += py;
    }

    Mat A = (Mat_<double>(3,3) <<   sum_xi_4,sum_xi_3,sum_xi_2,
             sum_xi_3,sum_xi_2,sum_xi_1,
             sum_xi_2,sum_xi_1,n);
    Mat b_sol = (Mat_<double>(3,1) << sum_right1,sum_right2,sum_right3);
    Mat solution = (Mat_<double>(3,1) << 0.0,0.0,0.0);

    solve( A, b_sol, solution);

    a = solution.at<double>(0,0);
    b = solution.at<double>(1,0);
    c = solution.at<double>(2,0);
}

