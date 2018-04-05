#include "fub_modelcar_tools/fub_modelcar_tools.h"
#include <vector>
#include <ros/ros.h>

using namespace fub_modelcar_tools;
int main() {

    // create class instance
   IO_pair steeringPair(0, 5.0f, 0.5f);
   std::vector <fub_modelcar_tools::IO_pair> SteeringPairs;
   SteeringPairs.push_back(steeringPair);
   SteeringPairs.push_back(IO_pair(180,5.0f, -0.5f));

//    // save data to archive
    std::string fileName="SteerAngleActuator.xml";
   saveXML(SteeringPairs,fileName.c_str());

    // ... some time later restore the class instance to its orginal state
    std::vector <IO_pair> newsteeringPairs;
    restoreXML(newsteeringPairs,"SteerAngleActuator.xml");
    for (int i=0;i<newsteeringPairs.size();i++)
        ROS_INFO_STREAM(i <<"-input command "<< newsteeringPairs.at(i).command<<", raduis= "<< newsteeringPairs.at(i).raduis<<" meters, output steering= "<<newsteeringPairs.at(i).steering<<" rad");

    return 0;
}

