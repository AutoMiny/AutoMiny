#include <hardware_calibration/HardwareCalibrationNodelet.h>

namespace hardware_calibration {

    HardwareCalibrationNodelet::HardwareCalibrationNodelet() : direction(Direction::FORWARD),
                                                               wantedDirection(Direction::FORWARD), currentSpeed(0.0) {
    }

    /** Nodelet initialization. Called by nodelet manager on initialization,
     ** can be used to e.g. subscribe to topics and define publishers.
     */
    void HardwareCalibrationNodelet::onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();
        steeringFeedbackBuffer = boost::circular_buffer<int16_t>(10);
        ticksBuffer = boost::circular_buffer<autominy_msgs::TickConstPtr>(10);

        configServer = boost::make_shared<dynamic_reconfigure::Server<HardwareCalibrationConfig> >(pnh);
        dynamic_reconfigure::Server<HardwareCalibrationConfig>::CallbackType f;
        f = boost::bind(&HardwareCalibrationNodelet::onReconfigure, this, _1, _2);
        configServer->setCallback(f);

        steeringPublisher = pnh.advertise<autominy_msgs::SteeringPWMCommand>("arduino/steering", 1);
        speedPublisher = pnh.advertise<autominy_msgs::SpeedPWMCommand>("arduino/speed", 1);
        calibratedSpeedPublisher = pnh.advertise<autominy_msgs::Speed>("carstate/calibrated_speed", 1);
        steeringAnglePublisher = pnh.advertise<autominy_msgs::SteeringAngle>("carstate/steering_angle", 1);
        speedMPSPublisher = pnh.advertise<autominy_msgs::SpeedCommand>("actuators/speed", 1);
        steeringFeedbackSubscriber = pnh.subscribe("arduino/steering_angle", 1,
                                                   &HardwareCalibrationNodelet::onSteeringFeedback, this,
                                                   ros::TransportHints().tcpNoDelay());
        speedSubscriber = pnh.subscribe("actuators/speed", 1, &HardwareCalibrationNodelet::onSpeedCommand, this,
                                        ros::TransportHints().tcpNoDelay());
        steeringSubscriber = pnh.subscribe("actuators/steering", 1, &HardwareCalibrationNodelet::onSteeringCommand,
                                           this, ros::TransportHints().tcpNoDelay());
        wantedSpeedSubscriber = pnh.subscribe("actuators/speed_normalized", 1,
                                              &HardwareCalibrationNodelet::onWantedSpeed, this,
                                              ros::TransportHints().tcpNoDelay());
        wantedSteeringSubscriber = pnh.subscribe("actuators/steering_normalized", 1,
                                                 &HardwareCalibrationNodelet::onWantedSteering, this,
                                                 ros::TransportHints().tcpNoDelay());
        ticksSubscriber = pnh.subscribe("arduino/ticks", 1, &HardwareCalibrationNodelet::onTicks, this,
                                        ros::TransportHints().tcpNoDelay());
    }

    void HardwareCalibrationNodelet::onTicks(const autominy_msgs::TickConstPtr& msg) {
        ticksBuffer.push_back(msg);

        if (ticksBuffer.size() > 1) {
            auto ticks = 0;
            for (const auto& tick : ticksBuffer) {
                ticks += tick->value;
            }
            autominy_msgs::Speed speedMsg;
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

            calibratedSpeedPublisher.publish(speedMsg);
            currentSpeed = speedMsg.value;
        }
    }

    void HardwareCalibrationNodelet::onSteeringFeedback(const autominy_msgs::SteeringFeedbackConstPtr& msg) {
        auto steeringFeedback = msg->value;
        steeringFeedbackBuffer.push_back(steeringFeedback);
        double average = std::accumulate(steeringFeedbackBuffer.begin(), steeringFeedbackBuffer.end(), 0.0) /
                         steeringFeedbackBuffer.size();

        if (average < config.minimum_steering_feedback || average > config.maximum_steering_feedback) {
            ROS_INFO("Steering feedback is not within range %i to %i: %f. Clamping!", config.minimum_steering_feedback,
                     config.maximum_steering_feedback, average);
            average = boost::algorithm::clamp(average, config.minimum_steering_feedback,
                                              config.maximum_steering_feedback);
        }

        auto radianSteering = mapRange(config.minimum_steering_feedback, config.maximum_steering_feedback,
                                       config.minimum_steering_radians, config.maximum_steering_radians, average);

        autominy_msgs::SteeringAngle steeringAngleMsg;
        steeringAngleMsg.header = msg->header;
        steeringAngleMsg.value = radianSteering;
        steeringAnglePublisher.publish(steeringAngleMsg);
    }

    void HardwareCalibrationNodelet::onWantedSpeed(const autominy_msgs::NormalizedSpeedCommandConstPtr& msg) {
        auto wantedSpeed = msg->value;

        if (wantedSpeed < 0.0) {
            wantedDirection = Direction::BACKWARD;
        } else if (wantedSpeed > 0.0) {
            wantedDirection = Direction::FORWARD;
        }

        if (wantedSpeed < -1.0 || wantedSpeed > 1.0) {
            ROS_INFO("Wanted speed is not within range -1.0 to 1.0: %f. Clamping!", wantedSpeed);
            wantedSpeed = boost::algorithm::clamp(wantedSpeed, -1.0, 1.0);
        }

        // function to go from [-1, 1] to m/s
        auto x = std::abs(wantedSpeed);
        auto mps = -2.3166083379533089e-002 * std::pow(x, 0)
                   + 1.3445417918423774e+000 * std::pow(x, 1)
                   + 3.8440484313852670e+000 * std::pow(x, 2)
                   - 3.3170433193228730e+000 * std::pow(x, 3);
        mps *= 2.0;

        // interpolation is not perfect so clip
        if (mps < 0) {
            mps = 0;
        }

        autominy_msgs::SpeedCommand speedMsg;
        speedMsg.header = msg->header;
        speedMsg.value = std::copysign(mps, msg->value);
        speedMPSPublisher.publish(speedMsg);
    }

    void HardwareCalibrationNodelet::onWantedSteering(const autominy_msgs::NormalizedSteeringCommandConstPtr& msg) {
        auto wantedSteering = msg->value;

        if (wantedSteering < -1.0 || wantedSteering > 1.0) {
            ROS_INFO("Wanted steering is not within range -1.0 to 1.0: %f. Clamping!", wantedSteering);
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

        autominy_msgs::SteeringPWMCommand steeringMsg;
        steeringMsg.header = msg->header;
        steeringMsg.value = static_cast<int16_t>(pwm);
        steeringPublisher.publish(steeringMsg);
    }

    void HardwareCalibrationNodelet::onSpeedCommand(const autominy_msgs::SpeedCommandConstPtr& msg) {
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

        auto command = boost::make_shared<autominy_msgs::SpeedPWMCommand>();
        command->header = msg->header;
        command->value = (std::copysign(normalized, msg->value) * 1000.0) / 2.0;
        speedPublisher.publish(command);
    }

    void HardwareCalibrationNodelet::onSteeringCommand(const autominy_msgs::SteeringCommandConstPtr& msg) {
        auto val = msg->value;
        if (msg->value < 0) {
            val /= config.minimum_steering_radians;
        } else {
            val /= config.maximum_steering_radians;
        }

        auto command = boost::make_shared<autominy_msgs::NormalizedSteeringCommand>();
        command->header = msg->header;
        command->value = val;
        this->onWantedSteering(command);
    }

    void HardwareCalibrationNodelet::onReconfigure(HardwareCalibrationConfig& config, uint32_t level) {
        this->config = config;
        ticksBuffer.set_capacity(static_cast<unsigned long>(config.number_of_ticks_filter));
        steeringFeedbackBuffer.set_capacity(static_cast<unsigned long>(config.number_of_steering_msgs_filter));
    }

    double HardwareCalibrationNodelet::mapRange(double a1, double a2, double b1, double b2, double s) {
        return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
    }


};