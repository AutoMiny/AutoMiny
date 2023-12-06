#include <hardware_calibration/HardwareCalibrationNodelet.h>

namespace hardware_calibration {


    /** Nodelet initialization. Called by nodelet manager on initialization,
     ** can be used to e.g. subscribe to topics and define publishers.
     */
    HardwareCalibrationNodelet::HardwareCalibrationNodelet(const rclcpp::NodeOptions& opts) : rclcpp::Node("hardware_calibration", opts),
    direction(Direction::FORWARD), wantedDirection(Direction::FORWARD), currentSpeed(0.0) {

        config.minimum_steering_feedback = declare_parameter<int>("minimum_steering_feedback", 192);
        config.maximum_steering_feedback = declare_parameter<int>("maximum_steering_feedback", 420);
        config.minimum_steering_radians = declare_parameter<double>("minimum_steering_radians", 0.512);
        config.maximum_steering_radians = declare_parameter<double>("maximum_steering_radians", -0.498);
        config.minimum_steering_pwm = declare_parameter<int>("minimum_steering_pwm", 950);
        config.maximum_steering_pwm = declare_parameter<int>("maximum_steering_pwm", 2150);
        config.minimum_speed_pwm = declare_parameter<int>("minimum_speed_pwm", -1000);
        config.maximum_speed_pwm = declare_parameter<int>("maximum_speed_pwm", 1000);
        config.ticks_to_m = declare_parameter<double>("ticks_to_m", 0.0027);
        config.number_of_ticks_filter = declare_parameter<int>("number_of_ticks_filter", 20);
        config.number_of_steering_msgs_filter = declare_parameter<int>("number_of_steering_msgs_filter", 10);
        config.direction_change_max_speed = declare_parameter<double>("direction_change_max_speed", 0.3);

        steeringFeedbackBuffer = boost::circular_buffer<int16_t>(config.number_of_steering_msgs_filter);
        ticksBuffer = boost::circular_buffer<autominy_msgs::msg::Tick::ConstSharedPtr>(config.number_of_ticks_filter);

        steeringPublisher = create_publisher<autominy_msgs::msg::SteeringPWMCommand>("arduino/steering", 1);
        speedPublisher = create_publisher<autominy_msgs::msg::SpeedPWMCommand>("arduino/speed", 1);
        calibratedSpeedPublisher = create_publisher<autominy_msgs::msg::Speed>("carstate/calibrated_speed", 1);
        steeringAnglePublisher = create_publisher<autominy_msgs::msg::SteeringAngle>("carstate/steering_angle", 1);
        speedMPSPublisher = create_publisher<autominy_msgs::msg::SpeedCommand>("actuators/speed", 1);

        steeringFeedbackSubscriber = create_subscription<autominy_msgs::msg::SteeringFeedback>("arduino/steering_angle", 10,
                                                           std::bind(&HardwareCalibrationNodelet::onSteeringFeedback, this, std::placeholders::_1));
        speedSubscriber = create_subscription<autominy_msgs::msg::SpeedCommand>("actuators/speed", 1, std::bind(&HardwareCalibrationNodelet::onSpeedCommand, this, std::placeholders::_1));
        steeringSubscriber = create_subscription<autominy_msgs::msg::SteeringCommand>("actuators/steering", 1, std::bind(&HardwareCalibrationNodelet::onSteeringCommand,
                                           this, std::placeholders::_1));
        wantedSpeedSubscriber = create_subscription<autominy_msgs::msg::NormalizedSpeedCommand>("actuators/speed_normalized", 1,
                                                      std::bind(&HardwareCalibrationNodelet::onWantedSpeed, this, std::placeholders::_1));
        wantedSteeringSubscriber = create_subscription<autominy_msgs::msg::NormalizedSteeringCommand>("actuators/steering_normalized", 1,
                                                         std::bind(&HardwareCalibrationNodelet::onWantedSteering, this, std::placeholders::_1));
        ticksSubscriber = create_subscription<autominy_msgs::msg::Tick>("arduino/ticks", 10, std::bind(&HardwareCalibrationNodelet::onTicks, this, std::placeholders::_1));
    }

    void HardwareCalibrationNodelet::onTicks(const autominy_msgs::msg::Tick::ConstSharedPtr& msg) {
        ticksBuffer.push_back(msg);

        if (ticksBuffer.size() > 1) {
            auto ticks = 0;
            for (const auto& tick : ticksBuffer) {
                ticks += tick->value;
            }
            autominy_msgs::msg::Speed speedMsg;
            speedMsg.header = msg->header;
            auto duration = ticksBuffer.size() * 0.01;
            if (duration == 0.0) {
                return;
            }
            speedMsg.value = (static_cast<double>(ticks) * config.ticks_to_m) / duration;

            // In case we want to switch directions, check if the current speed allows for a direction change
            // since the car might still be braking
            if (wantedDirection != direction && speedMsg.value <= config.direction_change_max_speed) {
                direction = wantedDirection;
            }

            speedMsg.value *= static_cast<double>(direction);

            calibratedSpeedPublisher->publish(speedMsg);
            currentSpeed = speedMsg.value;
        }
    }

    void HardwareCalibrationNodelet::onSteeringFeedback(const autominy_msgs::msg::SteeringFeedback::ConstSharedPtr& msg) {
        auto steeringFeedback = msg->value;
        steeringFeedbackBuffer.push_back(steeringFeedback);
        double average = std::accumulate(steeringFeedbackBuffer.begin(), steeringFeedbackBuffer.end(), 0.0) /
                         steeringFeedbackBuffer.size();

        if (average < config.minimum_steering_feedback || average > config.maximum_steering_feedback) {
            RCLCPP_INFO(get_logger(), "Steering feedback is not within range %i to %i: %f. Clamping!", config.minimum_steering_feedback,
                     config.maximum_steering_feedback, average);
            average = boost::algorithm::clamp(average, config.minimum_steering_feedback,
                                              config.maximum_steering_feedback);
        }

        auto radianSteering = mapRange(config.minimum_steering_feedback, config.maximum_steering_feedback,
                                       config.minimum_steering_radians, config.maximum_steering_radians, average);

        autominy_msgs::msg::SteeringAngle steeringAngleMsg;
        steeringAngleMsg.header = msg->header;
        steeringAngleMsg.value = radianSteering;
        steeringAnglePublisher->publish(steeringAngleMsg);
    }

    void HardwareCalibrationNodelet::onWantedSpeed(const autominy_msgs::msg::NormalizedSpeedCommand::ConstSharedPtr& msg) {
        auto wantedSpeed = msg->value;

        if (wantedSpeed < 0.0) {
            wantedDirection = Direction::BACKWARD;
        } else if (wantedSpeed > 0.0) {
            wantedDirection = Direction::FORWARD;
        }

        if (wantedSpeed < -1.0 || wantedSpeed > 1.0) {
            RCLCPP_INFO(get_logger(), "Wanted speed is not within range -1.0 to 1.0: %f. Clamping!", wantedSpeed);
            wantedSpeed = boost::algorithm::clamp(wantedSpeed, -1.0, 1.0);
        }

        // function to go from [-1, 1] to m/s
        auto x = std::abs(wantedSpeed);
        auto mps = -2.3166083379533089e-002 * std::pow(x, 0)
                   + 1.3445417918423774e+000 * std::pow(x, 1)
                   + 3.8440484313852670e+000 * std::pow(x, 2)
                   - 3.3170433193228730e+000 * std::pow(x, 3);
        //mps *= 2.0;

        // interpolation is not perfect so clip
        if (mps < 0) {
            mps = 0;
        }

        autominy_msgs::msg::SpeedCommand speedMsg;
        speedMsg.header = msg->header;
        speedMsg.value = std::copysign(mps, msg->value);
        speedMPSPublisher->publish(speedMsg);
    }

    void HardwareCalibrationNodelet::onWantedSteering(const autominy_msgs::msg::NormalizedSteeringCommand::ConstSharedPtr& msg) {
        auto wantedSteering = msg->value;

        if (wantedSteering < -1.0 || wantedSteering > 1.0) {
            RCLCPP_INFO(get_logger(), "Wanted steering is not within range -1.0 to 1.0: %f. Clamping!", wantedSteering);
            wantedSteering = boost::algorithm::clamp(wantedSteering, -1.0, 1.0);
        }

        // left and right have different limits
        auto pwm = 0.0;
        if (wantedSteering < 0) {
            pwm = mapRange(-1.0, 0, config.maximum_steering_radians, 0, wantedSteering);
        } else {
            pwm = mapRange(1.0, 0, config.minimum_steering_radians, 0, wantedSteering);
        }
        pwm = mapRange(config.minimum_steering_radians, config.maximum_steering_radians, config.minimum_steering_pwm,
                       config.maximum_steering_pwm, pwm);

        autominy_msgs::msg::SteeringPWMCommand steeringMsg;
        steeringMsg.header = msg->header;
        steeringMsg.value = static_cast<int16_t>(pwm);
        steeringPublisher->publish(steeringMsg);
    }

    void HardwareCalibrationNodelet::onSpeedCommand(const autominy_msgs::msg::SpeedCommand::ConstSharedPtr& msg) {
        if (msg->value < 0.0) {
            wantedDirection = Direction::BACKWARD;
        } else if (msg->value > 0.0) {
            wantedDirection = Direction::FORWARD;
        }

        // The mapping is symmetric so interpolate the absolute value
        auto x = std::abs(msg->value);

        auto normalized = 1.5136448288272340e-002 * pow(x, 0)
                          + 7.2086980469484707e-001 * pow(x, 1)
                          + -5.7014814155497762e-001 * pow(x, 2)
                          + 2.3646519448237449e-001 * pow(x, 3);

        // interpolation is not perfect so clip
        if (normalized < 0) {
            normalized = 0;
        }

        autominy_msgs::msg::SpeedPWMCommand command;
        command.header = msg->header;
        command.value = (std::copysign(normalized, msg->value) * 1000.0) / 2.0;
        speedPublisher->publish(command);
    }

    void HardwareCalibrationNodelet::onSteeringCommand(const autominy_msgs::msg::SteeringCommand::ConstSharedPtr& msg) {
        auto val = msg->value;
        if (msg->value < 0) {
            val /= config.minimum_steering_radians;
        } else {
            val /= -config.maximum_steering_radians;
        }

        auto command = std::make_shared<autominy_msgs::msg::NormalizedSteeringCommand>();
        command->header = msg->header;
        command->value = val;
        this->onWantedSteering(command);
    }

    double HardwareCalibrationNodelet::mapRange(double a1, double a2, double b1, double b2, double s) {
        return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
    }


};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hardware_calibration::HardwareCalibrationNodelet)
