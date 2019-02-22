#include <utility>
#include <ros/ros.h>
#include <hd_map/Polyline3D.h>


namespace hd_map {
    std::vector<tf2::Vector3> Polyline3D::getPoints() {
        return this->points;
    }

    Polyline3D::Polyline3D(std::vector<tf2::Vector3>& points) : points(points), cachedLength(0) {

    }

    double Polyline3D::length() {
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

    tf2::Vector3 Polyline3D::interpolate(double param) {
        if (param < 0 || param > length()) {
            ROS_ERROR("Interpolation is out of range");
        }

        tf2::Vector3 p;

        double length = 0;
        // Find the line part we want to interpolate at
        for (int i = 0; i < this->points.size() - 1; i++) {
            auto p1 = points.at(i);
            auto p2 = points.at(i + 1);

            double current = p1.distance(p2);

            if (length + current >= param) {
                double partParam = param - length;
                double ratio = partParam / current;
                p = p2 * ratio + p1 * (1 - ratio);

                return p;
            }

            length += current;
        }

        return p;
    }

    double Polyline3D::findClosestParameter(const tf2::Vector3& p) {
        auto closestParam = DBL_MAX;
        auto currentDistance = DBL_MAX;

        auto length = 0.0;
        for (int i = 0; i < this->points.size() - 1; i++) {
            auto p1 = points.at(i);
            auto p2 = points.at(i + 1);

            double current = p1.distance(p2);

            tf2::Vector3 AP, AB;
            AP = p - p1;
            AB = p2 - p1;

            double lengthSqrAB = AB.length2();
            double nx = AP.dot(AB) / lengthSqrAB;

            if (nx < 0) {
                nx = 0;
            }

            if (nx > 1) {
                nx = 1;
            }

            tf2::Vector3 point = interpolate (length + current * nx);
            auto distance = point.distance(p);

            if (distance < currentDistance) {
                currentDistance = distance;
                closestParam = length + current * nx;
            }

            length += current;
        }

        return closestParam;
    }

    tf2::Vector3 Polyline3D::gradient(double param) {
        double length = 0;
        // Find the line part we want to interpolate at
        for (int i = 0; i < this->points.size() - 1; i++) {
            auto p1 = points.at(i);
            auto p2 = points.at(i + 1);

            double current = p1.distance(p2);

            if (length + current >= param) {
                return p2 - p1;
            }

            length += current;
        }

        return {};
    }

    void Polyline3D::reverse() {
        std::reverse(this->points.begin(), this->points.end());
    }
}
