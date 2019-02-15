#pragma once

#include <memory>
#include <geometry_msgs/Point.h>
#include <tf2/utils.h>

namespace hd_map {
    class Polyline2D {
    public:
        explicit Polyline2D(std::vector<tf2::Vector3>& points);
        const std::vector<tf2::Vector3>& getPoints();
        tf2::Vector3 interpolate(double param);
        double length();
    private:
        double cachedLength;
        std::vector<tf2::Vector3> points;
    };
}

