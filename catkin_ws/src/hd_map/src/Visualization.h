#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hd_map/HDMap.h"

namespace hd_map {
    class Visualization {
    public:
        explicit Visualization(ros::NodeHandle& nh);
        void visualize();
        void onClickedPoint(const geometry_msgs::PointStampedConstPtr& point);
    private:
        HDMap map;
        rclcpp::Publisher<>::SharedPtr roadsPublisher;
        rclcpp::Publisher<>::SharedPtr lanesPublisher;
        rclcpp::Publisher<>::SharedPtr routePublisher;
        rclcpp::Subscription<>::SharedPtr mapOriginSubscriber;
        rclcpp::Subscription<>::SharedPtr clickedPointSubscriber;
        geometry_msgs::PointStampedConstPtr start, end;
    };
}
