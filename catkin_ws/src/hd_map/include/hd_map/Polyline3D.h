#pragma once

#include <memory>
#include <geometry_msgs/Point.h>
#include <tf2/utils.h>

namespace hd_map {
    class Polyline3D {
    public:
        explicit Polyline3D(std::vector<tf2::Vector3>& points);
        std::vector<tf2::Vector3> getPoints();
        tf2::Vector3 interpolate(double param);
        double length();
        double findClosestParameter(const tf2::Vector3& p);
        tf2::Vector3 gradient(double param);
        void reverse();
    private:
        double cachedLength;
        std::vector<tf2::Vector3> points;
    };
}

