//
//  LaneModel.h
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

#ifndef __LaneDetection__LaneModel__
#define __LaneDetection__LaneModel__

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include "PointSet.h"
#include <pthread.h>
#include "Circle.h"

using namespace std;
using namespace cv;


class LaneModel{
public:


    LaneModel(bool doReject, int image_w_half_, int lane_width_);

    int getCarState(double* curvature,double* distanceRightLane,double* angle,bool* isCurve);

    void preGroupedUpdate(vector<vector<Point2d> >* points);
    void applyCarMovement(double dy,double angle_rad);

    void improvedUpdate(vector<vector<Point2d> >* preGroupedPoints, bool midLaneFound);
    void applyCarMovementImproved(double dy,double angle_rad);

    //parabola parameters of the result
    vector<double> a_hist;
    vector<double> b_hist;
    vector<double> c_hist;    

    //parabola parameters used for grouping the new points
    vector<double> a_hist_grouping;
    vector<double> b_hist_grouping;
    vector<double> c_hist_grouping;
    vector<double> validity_y_begin;
    vector<double> validity_y_end;

    int getCurrentLane();

    void getDebugImage(Mat laneModelDrawing);
    vector<double> getHistoryParabola();
    vector<double> getGroupingParabola();



    
//private:
    
    bool modelLocked;

    int certainty;

    vector<Point2d> streetHistory;
    vector<Point2d> currentStreetEstimate;
        
    void modelGroupLaneMapping(vector<PointSet>* groups);
    void modelGroupLaneMappingImproved(vector<PointSet>* groups);
    void groupLaneMappingMidFound(vector<PointSet>* groups);

    int nrBadFits;
    void updateHistory();
    
    void estimateLane();

    void changeLane();
    bool lockRightLane;

    bool rejectFirstFrames;

    double laneWidth;

    pthread_mutex_t streetHistoryMutex;

    int counter;

    double yAcc;
    double angAcc;

    int badFitNumber;

    void initialize(bool doReject,bool doTurnLeft);

    int isCurveCounter;

    vector<double> fitted_a;
    vector<double> fitted_b;
    vector<double> fitted_c;


    RNG rand;

    int image_w_half;
};

#endif /* defined(__LaneDetection__LaneModel__) */
