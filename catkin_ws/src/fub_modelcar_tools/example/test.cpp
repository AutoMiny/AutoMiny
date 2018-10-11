#include "fub_modelcar_tools/fub_modelcar_tools.h"
#include "fub_modelcar_tools/Object3DPoints.h"


using namespace fub_modelcar_tools;
int main() {
    // create string class instance
    SteeringCalibrationData steeringPair(0, 1.034769503609362f, -0.25398560514106894f, 200);
    std::vector <SteeringCalibrationData> SteeringPairs;
    SteeringPairs.push_back(steeringPair);
    SteeringPairs.push_back(SteeringCalibrationData(30, 1.6414565448038825f, -0.15906583908123229f,238));
    SteeringPairs.push_back(SteeringCalibrationData(60, 4.43828893936895f, -0.05861469201518021f,272));
    SteeringPairs.push_back(SteeringCalibrationData(90, 4.930509500835628f, -0.05275735712052749f,310));
    SteeringPairs.push_back(SteeringCalibrationData(120,1.7373222925767673f, 0.15021993496784733f,344));
    SteeringPairs.push_back(SteeringCalibrationData(150,1.1127758201182651f, 0.2358298402182585f,378));
    SteeringPairs.push_back(SteeringCalibrationData(180,0.7591443474761536f, 0.3495668217104655f,400));

   // save data to archive
    std::string fileName="SteerAngleActuator.xml";
   saveXML(SteeringPairs,fileName.c_str());

    // ... some time later restore the class instance to its orginal state
    std::vector <SteeringCalibrationData> newSteeringPairs;
    restoreXML(newSteeringPairs,fileName.c_str());
    for (int i=0;i<newSteeringPairs.size();i++)
        std::cout<<i <<"-input command "<< newSteeringPairs.at(i).command<<", raduis= "<< newSteeringPairs.at(i).raduis
        <<" meters, output steering= "<<newSteeringPairs.at(i).steering<<" rad, feedback ="<<newSteeringPairs.at(i).feedback<<std::endl;

    std::cout<<"_______________"<<std::endl;
    // create speed class instance
    SpeedCalibrationData speedPair(100,14.5201254152720072);
    std::vector <SpeedCalibrationData> SpeedPairs;
    SpeedPairs.push_back(speedPair);
    SpeedPairs.push_back(SpeedCalibrationData(150, 50.975763494817373f));
    SpeedPairs.push_back(SpeedCalibrationData(200, 80.335490886981631f));
    SpeedPairs.push_back(SpeedCalibrationData(250, 104.46216179342825f));
    SpeedPairs.push_back(SpeedCalibrationData(300, 130.82557839894985f));
    SpeedPairs.push_back(SpeedCalibrationData(350, 155.70703543164225f));
    SpeedPairs.push_back(SpeedCalibrationData(400, 181.78574816576872f));

   // save data to archive
   fileName="SpeedActuator.xml";
   saveXML(SpeedPairs,fileName.c_str());

    // ... some time later restore the class instance to its orginal state
    std::vector <SpeedCalibrationData> newSpeedPairs;
    restoreXML(newSpeedPairs,fileName.c_str());
    for (int i=0;i<newSpeedPairs.size();i++)
        std::cout<<i <<"-input command "<< newSpeedPairs.at(i).command<<", twist= "<< newSpeedPairs.at(i).twist <<std::endl;

    std::cout<<"_______________"<<std::endl;
    //create camera class instance
    cv::Mat intrinsicMatrix;
    cv::Mat distCoeffs;
    intrinsicMatrix = (cv::Mat1d(3, 3) << 616.9697, 0, 323.7197, 0, 617.4253, 244.3983, 0, 0, 1);
    distCoeffs = (cv::Mat1d(1, 4) << 0,0,0,0);
    CameraCalibrationData cameraPair(intrinsicMatrix,distCoeffs, 17.8, 18);
    std::vector <CameraCalibrationData> cameraPairs;
    cameraPairs.push_back(cameraPair);
   // save data to archive
   fileName="CameraParams.xml";
   saveXML(cameraPairs,fileName.c_str());

    // ... some time later restore the class instance to its orginal state
    std::vector <CameraCalibrationData> newCameraPairs;
    restoreXML(newCameraPairs,fileName.c_str());
    for (int i=0;i<newCameraPairs.size();i++)
        std::cout<<"intrinsicMatrix= \n"<< newCameraPairs.at(i).intrinsicMatrix
                    <<"\n distCoeffs= "<<newCameraPairs.at(i).distCoeffs
                    <<"\n pitch= "<< newCameraPairs.at(i).pitch 
                    <<"\n height= "<< newCameraPairs.at(i).height <<std::endl;

    std::cout<<"_______________"<<std::endl;
    // create Object3DPoints class instance
    Object3DPoints point_3d(cv::Point3f(0,0,0));
    std::vector <Object3DPoints> points_3d;
    points_3d.push_back(point_3d);
    points_3d.push_back(Object3DPoints(cv::Point3f(0,5.5,0)));
    points_3d.push_back(Object3DPoints(cv::Point3f(0,11.0,0)));
    points_3d.push_back(Object3DPoints(cv::Point3f(12.8,0,0)));
    points_3d.push_back(Object3DPoints(cv::Point3f(12.8,5.5,0)));
    points_3d.push_back(Object3DPoints(cv::Point3f(12.8,11.0,0)));

   // save data to archive
   fileName="Object3DPoints.xml";
   saveXML(points_3d,fileName.c_str());

    // ... some time later restore the class instance to its orginal state
    std::vector <Object3DPoints> newPoints;
    restoreXML(newPoints,fileName.c_str());
    for (int i=0;i<newPoints.size();i++)
        std::cout<<i <<"- "<< newPoints.at(i).point3d<<std::endl;

    std::cout<<"_______________"<<std::endl;

    return 0;
}
