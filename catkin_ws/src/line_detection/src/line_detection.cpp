#include "stdafx.h"
#include "line_detection.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv/cv.h"
#include <stdio.h>
#include <math.h>

#ifndef DEBUG
#define DEBUG
Mat dbg;
#endif

using namespace std;
using namespace cv;


LineDetection::LineDetection(const tChar* __info) : cFilter(__info) {
    m_thresholdvalue=155;
    m_lineNum = 6;
    m_carCenterPoint = 340;
    m_topScanline = 330;
    m_bottomScanline = 450; 
    m_lineDifference = 20;  
    m_minDistance = 5;
    m_maxDistance = 60;
    m_lastSteeringAngle = 0;
    m_predictedLanePoint = m_carCenterPoint; 
    m_predictedStartPoint = m_carCenterPoint;
    m_minLanePoints = 3;
    m_maxLaneWidth = 25;

    for (int i = 0; i < m_lineNum; ++i) {
        m_lanePoints.push_back(Point(-1, -1));
        m_lastLeftPoints.push_back(Point(-1, -1));
        m_lastRightPoints.push_back(Point(-1, -1));
        m_lastLanePoints.push_back(Point(-1, -1));
    }
}

int LineDetection::ProcessInput(Mat img) {
    
        //cut it
        Mat image;
        int padding = 10;
        image = img(cv::Range(m_topScanline-padding,m_bottomScanline+padding), 
                cv::Range(0, src.cols)).clone();
#ifdef DEBUG
        dbg = image.clone();
#endif
        //reset values
        int cut_topScanline = padding;
        int cut_bottomScanline = image.rows - padding;

        //make grey/white picture
        cvtColor(image, grey, CV_RGB2GRAY);

        //Treshold to get binary picture
        threshold(grey, greythresh, m_thresholdvalue, 500,THRESH_BINARY);

        //find edges
        Canny(greythresh, linecanny, 0, 2, 7, false);
        Size s = linecanny.size();
            
        //this is the computed center between the driving lines on the next scanline
        m_predictedLanePoint = m_predictedStartPoint;
        //this is a counter for the lines
        int line_counter = 0;
        //go over all coloumns of every scanline
        //find a white pixel to each side and predict the center of the driving lane on the 
        for (int y = cut_bottomScanline; y > cut_topScanline; y -= m_lineDifference) {
#ifdef DEBUG
            //print the predicted center of the lane
            circle(dbg, 
                Point(m_predictedLanePoint, y), 
                5, 
                Scalar(255, 100, 100), 
                -1, 
                CV_AA);
#endif
            //these signal that we found a point
            bool foundLeft = false, foundRight = false;
            //these signal that we found a candidate and we are looking for an endpoint
            bool foundLeftMaybe = false, foundRightMaybe = false;
            //these are the points of the right line and the middle line
            Point left, right;

            //find middle line
            for (int x = m_predictedLanePoint - 1; x > 0; --x) {
                //we got a candidate and try to confirm it
                if (foundLeftMaybe){
                    int dist = left.x - x;
                    //we found a point but it is too close
                    if (linecanny.at<uchar>(y, x)>254 && dist <= m_minDistance) {
                        //set new candidate
                        left = Point(x, y);
//LOG_INFO(cString::Format("decline on %i : %i",line_counter, dist));
                        
                    }else if(linecanny.at<uchar>(y, x) > 254 
                            && dist >= m_minDistance 
                            && dist < m_maxDistance){

                        //we can confirm our candidate!
                        foundLeft = true;
#ifdef DEBUG
                        //pint found left point
                        circle(dbg, left, 5, Scalar(100, 100, 255), -1, CV_AA);
#endif  
                        //stop searching
                        break;
                    }else if (dist > m_maxDistance){
                        //we didnt find a point and we are too far away, so we decline the candidate
                        foundLeftMaybe = false;
                    }
                }else if(linecanny.at<uchar>(y, x) > 254){
                    //we found a point and dont have a candidate, so set candidate
                    foundLeftMaybe = true;
                    left = Point(x, y);
                }
            }

            //find right line
            for (int x = m_predictedLanePoint; x < s.width - 1; ++x) {
                //we got a candidate and try to confirm it
                if (foundRightMaybe){
                    int dist_to_point = x - right.x;
                    //we found a point but it is too close
                    if (linecanny.at<uchar>(y, x)>254 
                            && dist_to_point <= m_minDistance) {
                        //set new candidate
                        right = Point(x, y);
                    }else if(linecanny.at<uchar>(y, x) > 254 
                            && dist_to_point > m_minDistance 
                            && dist_to_point < m_maxDistance){
                        //we can confirm our candidate!
                        foundRight = true;
#ifdef DEBUG
                        circle(dbg, 
                            right, 
                            5, 
                            Scalar(100, 100, 255), 
                            -1, 
                            CV_AA);
#endif  
                        //stop searching
                        break;
                    }else if (dist_to_point > m_maxDistance){
                        //we didnt find a point and we are too far away, so we decline the candidate
                        foundRightMaybe = false;
                    }
                }else if(linecanny.at<uchar>(y, x) > 254){
                    //we found a point and dont have a candidate, so set candidate
                    foundRightMaybe = true;
                    right = Point(x, y);
                }
            }

            //Some helping vars. They are always relative to the m_predictedLanePoint which is the mid point.
            int dist_to_left_line = m_predictedLanePoint - left.x;
            int dist_to_right_line = right.x - m_predictedLanePoint;
            int predicted_left_line = m_predictedLanePoint + (-340 + dist_to_right_line);


            //we got a left and a right point, go validate them
            if (foundLeft && foundRight) {
                /*//prevent left lane dectection by detecting a too far distance to the right
                if (dist_to_left_line > 200) {
                    m_predictedLanePoint = 3.0/4*right.x;
                    m_lanePoints.at(line_counter) = Point(m_predictedLanePoint, y); 
                    m_lastRightPoints.at(line_counter) = right;
                    m_lastLeftPoints.at(line_counter) = left;
#ifdef DEBUG
                    circle(dbg, 
                        Point(m_predictedLanePoint, left.y), 
                        5, 
                        Scalar(100, 100, 255), 
                        -1, 
                        CV_AA);
#endif

                } else {*/
                    ComputeRegularCenter(left, right, line_counter);
                //}
            } else if (foundRight) {
                // we found only a right point, so mirror directly
                m_predictedLanePoint = MirrorRightLine(right, 
                    dist_to_right_line, 
                    predicted_left_line, 
                    line_counter);
            } else if (foundLeft) {
                    int predicted_right_line;
                    predicted_right_line= m_predictedLanePoint + (340 - dist_to_left_line);
                    if (predicted_right_line>640) 
                            predicted_right_line=640;
                //we found only a left point, so miror directly
                int compute_x = (predicted_right_line+left.x)/2.0;
#ifdef DEBUG
                //print predicted right line
                circle(dbg, 
                    Point(predicted_right_line, left.y), 
                    5, 
                    Scalar(100, 100, 255), 
                    -1, 
                    CV_AA);
#endif
                //shift the predicted lane          
                if(line_counter > 0 && m_lastLeftPoints.at(line_counter-1).x != -1) {
                    int last_x = m_lastLeftPoints.at(line_counter-1).x;
                    int diff_left_last_now = last_x - left.x;
                    compute_x -= (diff_left_last_now);
                    
                }

                //save infos
                m_lanePoints.at(line_counter) = Point(compute_x, left.y);
                m_lastLeftPoints.at(line_counter) = left;
                m_lastRightPoints.at(line_counter) = Point(predicted_right_line, left.y);
                m_predictedLanePoint = compute_x;
            } else {
                //no point on the line
                m_lanePoints.at(line_counter) = Point(-1, y);
                m_lastLeftPoints.at(line_counter) = Point(-1, y);
                m_lastRightPoints.at(line_counter) = Point(-1, y);
                m_lastLanePoints.at(line_counter) = Point(-1, y);       
            }
            line_counter++;
#ifdef DEBUG
            line(dbg, 
                Point(0, y), 
                Point((s.width - 1), y), 
                Scalar(255, 0, 0), 
                1,
                    CV_AA);
#endif
        }
#ifdef DEBUG

        //print debug m_lanePoints and a path
        PrintDebugPathPoints();
        
#endif
        //save lane points
        for(int i=0;i<m_lineNum; ++i) {
            if(m_lanePoints.at(i).x != -1) {
                m_lastLanePoints.at(i) = m_lanePoints.at(i);
            } else {
#ifdef DEBUG
                //LOG_INFO(cString::Format("Point: %i not found", i));
#endif
            }
        }

        //the number of valid m_lanePoints
        int found = CountPointsFound();

        //compute the mean point
        float mean = ComputeMean(found);
        
        //shift start point for the next picture
        int dist_start_to_mean = mean - m_predictedStartPoint;
        m_predictedStartPoint = m_carCenterPoint+ 0.25 * dist_start_to_mean;

        //compute an angle if there are more than 2 points found
        if(found >= m_minLanePoints){
#ifdef DEBUG
            line(dbg, 
                Point(mean, cut_topScanline), 
                Point(320, s.height), 
                Scalar(0, 0, 255), 
                1, 
                CV_AA);
#endif
            float angle = ComputeSteeringAngle(mean, s, cut_topScanline);
            if (angle!=m_lastSteeringAngle)
                OutputSteering(angle);
            //save steering angle in case we cant find enough points
            m_lastSteeringAngle = angle;
        } else{
            //LOG_INFO("Less than 3 points found");     
            //OutputSteering(m_lastSteeringAngle);
        }
        
        //DetectStopLine(image);

    
#if defined(DEBUG)          
            
        //print image
        imshow("linecanny", linecanny);
        imshow("lane", dbg);
        waitKey(1);
            
#endif
    }
    return 1;         
}
int LineDetection::DetectStopLine(Mat &image) {
    int whites = 0;
    for(int i = 1; i<image.cols; ++i) {
        if (image.at<uchar>(image.rows-10, i) > 200) {
            whites++;
        }
    }



    //LOG_INFO(cString::Format("whites: %i", whites));

    return 1;

}

int LineDetection::ComputeRegularCenter(Point &left, Point &right, int line_counter) {

    //compute the center between left and right
    int compute_x = (left.x + right.x) / 2;
    m_lastLeftPoints.at(line_counter) = left;
    m_lastRightPoints.at(line_counter) = right;

    //shift the lane 
    int last_left_x = m_lastLeftPoints.at(line_counter).x;
    int last_right_x = m_lastRightPoints.at(line_counter).x;

    if (abs(left.x - last_left_x) > abs(last_right_x - right.x)) {
        compute_x += (2.0/3)*(left.x - last_left_x);
    } else {
        compute_x += (2.0/3)*(right.x - last_right_x);
    }

    m_lanePoints.at(line_counter) = Point(compute_x, left.y);
    m_predictedLanePoint = compute_x;

    return 1;
}


float LineDetection::ComputeSteeringAngle(float mean, Size &s, int cut_topScanline) {
    //compute the angle of the lines
    float angle = 0;
    //check for a right angle
        int carPosition=340;
    if(mean-carPosition != 0){
        //compute the angle by using sinus triangle stuff
        double adjacentSide = sqrt(pow(mean - carPosition, 2));
        int oppositeLeg = s.height - cut_topScanline;
        double alpha = oppositeLeg / (adjacentSide*1.0);
        //multiply to get degrees
        double degree = 180/3.14159265;
        angle = atan(alpha);
        angle = angle * degree;
    }else{  
        angle = 90;
    }
    //transform angle of the lines to steering angle
    angle = 90 - angle;

    //set to singed or unsingd, left is negative
    if (mean < carPosition) {
        angle *= (-1);
    }
//LOG_INFO(cString::Format("angle: %f", angle));

    return angle;
}

int LineDetection::PrintDebugPathPoints() {
    for (int i = 1; i < m_lineNum; ++i) {
        if (m_lanePoints.at(i).x != -1 
                && m_lanePoints.at(i-1).x != -1)

            line(dbg, 
                m_lanePoints.at(i-1), 
                m_lanePoints.at(i), 
                Scalar(0, 255, 0), 
                1, 
                CV_AA);
        }

        for (int i = 0; i < m_lineNum; ++i) {
            if (m_lanePoints.at(i).x != -1) {
                circle(dbg, 
                    m_lanePoints.at(i), 
                    5, 
                    Scalar(0, 0, 255), 
                    -1, 
                    CV_AA);
            } 
        }
    return 1;
}

float LineDetection::ComputeMean(int found) {
    float mean  = 0;
    
    //sum up and divide through the number of existing m_lanePoints
    for(int i = 0; i<m_lineNum; ++i){
        //only if we found a valid point
        if(m_lanePoints.at(i).x != -1){
            mean += m_lanePoints.at(i).x;
        }
    }
    mean = mean/found;
    
    return mean;
}

int LineDetection::CountPointsFound() {
    int found = 0;
    
    for(int i = 0; i<m_lineNum; ++i){
        if(m_lanePoints.at(i).x != -1){
            found++;
        }
    }
    
    return found;
}

int LineDetection::MirrorRightLine(Point &right, 
        int dist_to_right_line, 
        int predicted_left_line, 
        int line_counter) {

#ifdef DEBUG
    //print predicted left point
    circle(dbg, 
        Point(predicted_left_line, right.y), 
        5, 
        Scalar(100, 100, 255), 
        -1, 
        CV_AA);

#endif
    int compute_x = (right.x + predicted_left_line) / 2;
    //shift predicted line if there is a valid point on the last line
    if(line_counter > 0 && m_lastRightPoints.at(line_counter-1).x != -1) {
        int last_x = m_lastRightPoints.at(line_counter-1).x;
        int diff_right_last_now = last_x - right.x;
        compute_x -= 2*(diff_right_last_now)/3.0;
    }
    m_lanePoints.at(line_counter) = Point(compute_x, right.y);

    m_lastRightPoints.at(line_counter) = right;
    m_lastLeftPoints.at(line_counter) = Point(predicted_left_line, right.y);

    return compute_x;
}

int LineDetection::ProcessFound() {        
    return 1;
}

int LineDetection::ProcessOutput() {
    return 1;
}

int LineDetection::UpdateImageFormat(const tBitmapFormat* pFormat) {
    if (pFormat != NULL) {
        m_sInputFormat = (*pFormat);        
        LOG_INFO(adtf_util::cString::Format(
            "EdgeDetection Filter Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
            m_sInputFormat.nWidth,
            m_sInputFormat.nHeight, 
            m_sInputFormat.nBytesPerLine, 
            m_sInputFormat.nSize, 
            m_sInputFormat.nPixelFormat));          

    }
    
    return 1;
}

void LineDetection::OutputSteering(float val) { 
    uint timeStamp = 0;  
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    int nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
   {   // focus for sample write lock
            //write date to the media sample with the coder of the descriptor
            __adtf_sample_write_lock_mediadescription( m_pCoderDescSignal,pMediaSample,pCoderOutput);

            pCoderOutput->Set("f32Value", (tVoid*)&(val));    
            pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }
    
    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oPinSteeringSignal.Transmit(pMediaSample);
}

tTimeStamp LineDetection::GetTime() {
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}
