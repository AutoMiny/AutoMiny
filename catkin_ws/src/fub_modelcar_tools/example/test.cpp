#include "fub_modelcar_tools/fub_modelcar_tools.h"
#include <vector>
#include <ros/ros.h>

using namespace fub_modelcar_tools;
int main() {

    // create class instance
    IO_pair steeringPair(0, 0.846490699486634f, 0.3121972894737317f);
    std::vector <fub_modelcar_tools::IO_pair> SteeringPairs;
    SteeringPairs.push_back(steeringPair);
    SteeringPairs.push_back(IO_pair(30, 0.9819587614192239f, 0.2679725611643332f));
    SteeringPairs.push_back(IO_pair(60, 1.6614453671514262f, 0.15713611273019293f));
    SteeringPairs.push_back(IO_pair(90, 5.1037043897833832f, 0.05096545011120175f));
    SteeringPairs.push_back(IO_pair(120,5.2721903721473739f, -0.04933537752660846f));
    SteeringPairs.push_back(IO_pair(150,2.1657968135458151f, -0.12033842917022389f));
    SteeringPairs.push_back(IO_pair(180,1.1260922599749981f, -0.23298915809989759f));

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

