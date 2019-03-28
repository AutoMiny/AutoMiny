#include <tick_calibration/TickCalibration.h>
#include <numeric>

namespace tick_calibration {
    TickCalibration::TickCalibration() : pastTrajectory(1000), pastTicks(1000) {}

    TickCalibration::~TickCalibration() = default;

    void TickCalibration::setConfig(tick_calibration::TickCalibrationConfig& config) {
        this->config = config;
        pastTrajectory.set_capacity(config.buffer_size);
        pastTicks.set_capacity(config.buffer_size);
    }

    void TickCalibration::addLocalization(const nav_msgs::OdometryConstPtr& odom) {
        pastTrajectory.push_back(odom);
    }

    void TickCalibration::addTick(const autominy_msgs::TickConstPtr& tick) {
        pastTicks.push_back(tick);
    }

    double TickCalibration::calibrate() {
        auto distance = 0.0;

        if (pastTrajectory.full() && pastTicks.full()) {
            for (int i = 0; i < pastTrajectory.size() - 1; ++i) {
                auto a = pastTrajectory[i];
                auto b = pastTrajectory[i + 1];

                distance += std::sqrt(std::pow(a->pose.pose.position.x - b->pose.pose.position.x, 2) +
                        std::pow(a->pose.pose.position.y - b->pose.pose.position.y, 2));
            }

            if (distance < config.min_length) {
                ROS_WARN("Distance smaller than min length needed for calibration");
                return -1;
            }

            auto ticks = 0;
            for (const auto& tick : pastTicks) {
                ticks += tick->value;
            }

            if (ticks < config.min_ticks) {
                ROS_WARN("Ticks less than min ticks needed for calibration");
                return -1;
            }

            ROS_INFO("Ticks to m: %f", distance / ticks);
            return distance / ticks;
        }

        return -1;
    }

}
