#include <vector>

class LineDetection
{
public:
		LineDetection(const tChar*);
		virtual ~LineDetection();

		/*! processing the image
		*/
		tResult ProcessFound();
		/*! processing the image
		*/


		tResult ProcessOutput();
		/*! changed a property
		@param strProperty char pointer to property
		*/
		tResult PropertyChanged(const char* strProperty);
		/*! updates the image format
		@param pFormat the new image format
		*/
		tResult UpdateImageFormat(const tBitmapFormat* pFormat);

		cv::Mat grey;				/* matrix for the gray image*/
		cv::Mat greythresh;			/* matrix for the gray threshold*/
		cv::Mat linecanny;			/* matrix for the canny lines*/

private:
		bool firstFrame;			/* flag for the first frame*/
		tBitmapFormat m_sInputFormat;		/* bitmap format of the input image*/
		
		//Properties:
		int m_thresholdvalue;			/* the threshold value*/
		int m_lineNum;				/* count of the scanlines*/
		void OutputSteering(tFloat32 val);
		cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
		tTimeStamp GetTime();
		/*! print pathpoints and all regular sidepoints*/
		tResult PrintDebugPathPoints();
		int MirrorRightLine(Point &right, 
				int dist_to_right_line, 
				int predicted_left_line, 
				int line_counter);

		int m_carCenterPoint;			/* the middle point of the car*/
		int m_topScanline;                   /* the first scanline from the top of the picture*/
                int m_bottomScanline;                /* the first scanline from the bottom of the picture*/
                int m_lineDifference;                /* the distance between each scanline*/
		int m_minDistance;			/* the minmial distance of the width of a line*/
		int m_maxDistance;			/* the maximal distance of the width of a line*/
		int m_lastSteeringAngle;		/* angle of the last picture*/
		std::vector<Point> m_lanePoints;		/* the points of the lane*/
		std::vector<Point> m_lastLeftPoints;		/* the points of the left line from last pic*/
		std::vector<Point> m_lastRightPoints;		/* the points of the right line from last pic*/
		std::vector<Point> m_lastLanePoints;		/* the points of the lane from the last pic*/
		int m_predictedLanePoint;			/* the predicted points of the lane*/
		int CountPointsFound();			/* counts, how many valid lane points are found*/
		float ComputeMean(int found);			/* compute the mean of all valid x values*/
		int m_minLanePoints;				/* the minimum number of valid points to compute a steering angle*/
		float ComputeSteeringAngle(tFloat mean, Size &s, int cut_topScanline);	/* compute the steering angle based on the mean of the x values*/
		int ValidateLanePoint(int computed_x, int predicted_x);	/* return 1 if the point is not too far away from the last one, -1 else*/
		int m_maxLaneWidth;				/* this value is used in validateLanePoint to validate by this distance*/
		tResult ComputeRegularCenter(Point &left, Point &right, int line_counter);	/* computes the lane point of to line points*/
		tResult DetectStopLine(Mat &image);		/* search for stop line using hough transformation*/
		int m_predictedStartPoint;			/* the point where to start searching*/
};
#endif 
