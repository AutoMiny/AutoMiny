/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**/
// Christoph Brickl, AEV:

/*! \brief cSpurerkennung
 *         
•	Zur Erkennung einer Spur wird das RGB Bild der Asus Xtion eingelesen und verarbeitet
•	Weiteres Vorgehen:
•	Zuschneiden des Orginal Bildes auf die eingestellte Größe
•	Erzeugen eines Graustufen Bildes
•	Anwenden eines Schwellwertfilters
•	Kantendedektion
•	Suchen nach Kanten auf den eingestellten cm-Marken
•	Auswerten der gefundenen Punkte ob sinnvolle Linie erstellt werden kann
•	Anzeigen der Linie mit Hilfe der GLC

 */

#ifndef _LaneDetection_FILTER_HEADER_
#define _LaneDetection_FILTER_HEADER_
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
using namespace std;
using namespace cv;

#include "LaneModel.h"
#include "LaneDetector.h"
#include "IPMapper.h"
#include "ContourModel.h"
#include "momenTUM_const.h"

class cLaneDetection
{
    private:
        // the node handle
        ros::NodeHandle nh_;

        // Node handle in the private namespace
        ros::NodeHandle priv_nh_;

        // subscribers
        ros::Subscriber read_images_;

        // publishers
        ros::Publisher publish_images;
        ros::Publisher publish_curvature;;

        double m_LastValue;

        bool firstFrame;               /**< flag for the first frame*/
        uint8_t  imagecount;              /**< counter for the imaes*/


        //define inner camera parameters (values will be gained from the opencv camera calibration program)
        /*float f_u;
        float f_v;

        //camera optical center
        float c_u;
        float c_v;

        //cos(pitch),cos(yaw),sin(pitch),sin(yaw)
        float c_1;
        float c_2;

        float s_1;
        float s_2;

        //heigth of the camera above ground(cm)
        float h;

        //Transformation matrix and inverse transformation matrix
        Mat T;
        Mat T_inv;

        //field of interest
        Point foe_p1;           //left top
        Point foe_p2;           //right bot
*/
        
        int cam_w;
        int cam_h;
        int proj_y_start;
        int proj_image_h;
        int proj_image_w;
        int proj_image_w_half;
        int roi_top_w;
        int roi_bottom_w;
        int proj_image_horizontal_offset;



        LaneDetector detector;
        LaneModel    model;

        //std::string PATH_2FEATURES,PATH_30FEATURES;

        // cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneInfo;
        // cObjectPtr<IMediaTypeDescription> m_pCoderDescMotionData;
        // cObjectPtr<IMediaTypeDescription> m_pCoderDescValue;
        // cObjectPtr<IMediaTypeDescription> m_pCoderDescValueOut;
        // cObjectPtr<IMediaTypeDescription> m_pCoderDescValueOvertaking;

    public:
        std::string classifier_file_path;

        void resetSystem();

    	cLaneDetection(ros::NodeHandle nh, int cam_w_, int cam_h_, int proj_y_start_,
            int proj_image_h_, int proj_image_w_, int roi_top_w_, int roi_bottom_w_, int roi_horizontal_offset_,
            int detector_size_, int lane_width, std::string path_2features, std::string path_30features);
    	virtual ~cLaneDetection();
        int Init();

    	/*! processing
    	@param pSample the input media sample

    	*/
        void ProcessInput(const sensor_msgs::Image::ConstPtr& msg);
        void pubRealSenseRGBImageMsg(cv::Mat& rgb_mat);
};

#endif 
