#include <emergency_stop/EmergencyStop.h>

emergency_stop::EmergencyStopNodelet::~EmergencyStopNodelet() = default;

emergency_stop::EmergencyStopNodelet::EmergencyStopNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("emergency_stop", opts) {
    config.angle_front = declare_parameter<double>("angle_front", 0.7);
    config.angle_back = declare_parameter<double>("angle_back", 0.7);
    config.brake_distance = declare_parameter<double>("brake_distance", 0.45);
    config.brake_distance_based_on_speed = declare_parameter<bool>("brake_distance_based_on_speed", false);
    config.reverse_minimum_distance = declare_parameter<double>("reverse_minimum_distance", 0.28);
    config.forward_minimum_distance = declare_parameter<double>("forward_minimum_distance", 0.07);
    config.negative_acceleration = declare_parameter<double>("negative_acceleration", 4.0);

    cb = add_on_set_parameters_callback(std::bind(&EmergencyStopNodelet::onConfig, this, std::placeholders::_1));

    speedPublisher = create_publisher<autominy_msgs::msg::SpeedPWMCommand>("speed", 1);
    scanSubscriber = create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&EmergencyStopNodelet::onScan, this, std::placeholders::_1));
    wantedSpeedSubscriber = create_subscription<autominy_msgs::msg::SpeedPWMCommand>("wanted_speed", 1, std::bind(&EmergencyStopNodelet::onWantedSpeed, this, std::placeholders::_1));
    currentSpeedSubscriber = create_subscription<autominy_msgs::msg::Speed>("carstate/speed", 1, std::bind(&EmergencyStopNodelet::onCurrentSpeed, this, std::placeholders::_1));
}

void emergency_stop::EmergencyStopNodelet::onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan) {
    double breakDistance = config.brake_distance;
    if (config.brake_distance_based_on_speed) {
        breakDistance = std::pow(currentSpeed, 2) / 2.0 * config.negative_acceleration;
    }

    emergencyStop = false;
    auto angleIncrement = scan->angle_increment;
    if (wantedSpeed >= 0) {    //forward.
        auto frontAngle = config.angle_front / 2.0;
        auto start = 0;
        auto end = static_cast<int>(frontAngle / angleIncrement);

        for (int i = 0; i < scan->ranges.size() && i < end; i++) {
            if (std::isfinite(scan->ranges[i]) && scan->ranges[i] <= breakDistance + config.forward_minimum_distance &&
                scan->ranges[i] > config.forward_minimum_distance) {
                emergencyStop = true;
                break;
            }
        }

        start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
        end = scan->ranges.size();
        for (int k = start; k < end; k++) {
            if (std::isfinite(scan->ranges[k]) && scan->ranges[k] <= breakDistance + config.forward_minimum_distance &&
                scan->ranges[k] > config.forward_minimum_distance) {
                emergencyStop = true;
                break;
            }
        }
    }

    if (wantedSpeed < 0) { //backward.
        auto backAngle = config.angle_back / 2.0;
        int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
        int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);
        for (int j = start; j < end && j < scan->ranges.size(); j++) {
            // we might see the camera in the laser scan
            if (std::isfinite(scan->ranges[j]) && scan->ranges[j] <= (breakDistance + config.reverse_minimum_distance) &&
                scan->ranges[j] > config.reverse_minimum_distance) {
                emergencyStop = true;
                break;
            }
        }
    }

    speedPublisher->publish(getSafeSpeed());
}

void emergency_stop::EmergencyStopNodelet::onCurrentSpeed(const autominy_msgs::msg::Speed::ConstSharedPtr &msg) {
    setCurrentSpeed(msg);
}

void
emergency_stop::EmergencyStopNodelet::onWantedSpeed(const autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr &msg) {
    setWantedSpeed(msg);
}

autominy_msgs::msg::SpeedPWMCommand emergency_stop::EmergencyStopNodelet::getSafeSpeed() {
    autominy_msgs::msg::SpeedPWMCommand msg;

    if (emergencyStop) {
        msg.value = 0;
    } else {
        msg.value = wantedSpeed;
    }

    return msg;
}

void emergency_stop::EmergencyStopNodelet::setCurrentSpeed(const autominy_msgs::msg::Speed::ConstSharedPtr &speed) {
    currentSpeed = speed->value;
}

void
emergency_stop::EmergencyStopNodelet::setWantedSpeed(const autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr &speed) {
    wantedSpeed = speed->value;
    speedPublisher->publish(getSafeSpeed());
}

rcl_interfaces::msg::SetParametersResult
emergency_stop::EmergencyStopNodelet::onConfig(const std::vector<rclcpp::Parameter> &params) {

    for (auto&& p : params) {
        const auto& name = p.get_name();

        if (name == "angle_front" && p.as_double() >= 0 && p.as_double() < 3.141) config.angle_front = p.as_double();
        if (name == "angle_back" && p.as_double() >= 0 && p.as_double() < 3.141) config.angle_back = p.as_double();
        if (name == "brake_distance" && p.as_double() >= 0 && p.as_double() < 2.0) config.brake_distance = p.as_double();
        if (name == "brake_distance_based_on_speed") config.brake_distance_based_on_speed = p.as_bool();
        if (name == "reverse_minimum_distance" && p.as_double() >= 0 && p.as_double() < 1.0) config.reverse_minimum_distance = p.as_double();
        if (name == "forward_minimum_distance" && p.as_double() >= 0 && p.as_double() < 1.0) config.forward_minimum_distance = p.as_double();
        if (name == "negative_acceleration" && p.as_double() >= 0 && p.as_double() < 10.0) config.negative_acceleration = p.as_double();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(emergency_stop::EmergencyStopNodelet);

