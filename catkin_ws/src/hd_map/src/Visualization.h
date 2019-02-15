#pragma once

#include <ros/ros.h>
#include "hd_map/HDMap.h"

namespace hd_map {
    class Visualization {
    public:
        explicit Visualization(ros::NodeHandle& nh);
        void visualize();
        void onClickedPoint(const geometry_msgs::PointStampedConstPtr& point);
    private:
        HDMap map;
        ros::Publisher roadsPublisher;
        ros::Publisher lanesPublisher;
        ros::Publisher routePublisher;
        ros::Subscriber mapOriginSubscriber;
        ros::Subscriber clickedPointSubscriber;
        geometry_msgs::PointStampedConstPtr start, end;
    };
}
