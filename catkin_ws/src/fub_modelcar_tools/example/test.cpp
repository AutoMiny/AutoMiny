#include "fub_modelcar_tools/fub_modelcar_tools.h"
#include <vector>
#include <ros/ros.h>

using namespace fub_modelcar_tools;
int main() {

    // create class instance
    IO_pair steeringPair(0, 1.034769503609362f, -0.25398560514106894f, 200);
    std::vector <fub_modelcar_tools::IO_pair> SteeringPairs;
    SteeringPairs.push_back(steeringPair);
    SteeringPairs.push_back(IO_pair(30, 1.6414565448038825f, -0.15906583908123229f,238));
    SteeringPairs.push_back(IO_pair(60, 4.43828893936895f, -0.05861469201518021f,272));
    SteeringPairs.push_back(IO_pair(90, 4.930509500835628f, -0.05275735712052749f,310));
    SteeringPairs.push_back(IO_pair(120,1.7373222925767673f, 0.15021993496784733f,344));
    SteeringPairs.push_back(IO_pair(150,1.1127758201182651f, 0.2358298402182585f,378));
    SteeringPairs.push_back(IO_pair(180,0.7591443474761536f, 0.3495668217104655f,400));

    

//    // save data to archive
    std::string fileName="SteerAngleActuator.xml";
   saveXML(SteeringPairs,fileName.c_str());

    // ... some time later restore the class instance to its orginal state
    std::vector <IO_pair> newsteeringPairs;
    restoreXML(newsteeringPairs,"SteerAngleActuator.xml");
    for (int i=0;i<newsteeringPairs.size();i++)
        ROS_INFO_STREAM(i <<"-input command "<< newsteeringPairs.at(i).command<<", raduis= "<< newsteeringPairs.at(i).raduis<<" meters, output steering= "<<newsteeringPairs.at(i).steering<<" rad, feedback ="<<newsteeringPairs.at(i).feedback);

    return 0;
}

