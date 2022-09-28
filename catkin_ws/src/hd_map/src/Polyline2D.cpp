#include <utility>
#include <hd_map/Polyline2D.h>
#include <rosconsole/macros_generated.h>
#include "rclcpp/rclcpp.hpp"

namespace hd_map {
    const std::vector<tf2::Vector3>& Polyline2D::getPoints() {
        return this->points;
    }

    Polyline2D::Polyline2D(std::vector<tf2::Vector3>& points) : points(points), cachedLength(0) {

    }

    double Polyline2D::length() {
        if (cachedLength > 0) {
            return cachedLength;
        }

        double length = 0;

        for (int i = 0; i < this->points.size() - 1; i++) {
            auto p1 = points.at(i);
            auto p2 = points.at(i + 1);


            length += p1.distance(p2);
        }
        cachedLength = length;

        return length;
    }

    tf2::Vector3 Polyline2D::interpolate(double param) {
        if (param < 0 || param > length()) {
            RCLCPP_ERROR(get_logger(), "Interpolation is out of range");
        }
        tf2::Vector3 p;

        double length = 0;
        // Find the line part we want to interpolate at
        for (int i = 0; i < this->points.size() - 1; i++) {
            auto p1 = points.at(i);
            auto p2 = points.at(i + 1);

            double current = p1.distance(p2);

            if (length + current > param) {
                double partParam = param - length;
                double ratio = partParam / current;
                p = p1 * ratio + p2 * (1 - ratio);

                return p;
            }

            length += current;
        }

        return p;
    }
}
