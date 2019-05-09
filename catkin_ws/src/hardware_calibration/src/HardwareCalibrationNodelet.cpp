#include <hardware_calibration/HardwareCalibrationNodelet.h>

namespace hardware_calibration {
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

            steeringPublisher = pnh.advertise<autominy_msgs::SteeringCommand>("arduino/steering", 1);
            speedPublisher = pnh.advertise<autominy_msgs::SpeedCommand>("arduino/speed", 1);
            calibratedSpeedPublisher = pnh.advertise<autominy_msgs::Speed>("carstate/calibrated_speed", 1);
            steeringAnglePublisher = pnh.advertise<autominy_msgs::SteeringAngle>("carstate/steering_angle", 1);
            steeringFeedbackSubscriber = pnh.subscribe("arduino/steering_angle", 1,
                                                      &HardwareCalibrationNodelet::onSteeringFeedback, this, ros::TransportHints().tcpNoDelay());
            wantedSpeedSubscriber = pnh.subscribe("control/normalized_wanted_speed", 1, &HardwareCalibrationNodelet::onWantedSpeed, this, ros::TransportHints().tcpNoDelay());
            wantedSteeringSubscriber = pnh.subscribe("control/normalized_wanted_steering", 1, &HardwareCalibrationNodelet::onWantedSteering, this, ros::TransportHints().tcpNoDelay());
            ticksSubscriber = pnh.subscribe("arduino/ticks", 1, &HardwareCalibrationNodelet::onTicks, this, ros::TransportHints().tcpNoDelay());
        }

        void HardwareCalibrationNodelet::onTicks(const autominy_msgs::TickConstPtr& msg) {
            ticksBuffer.push_back(msg);

            if (ticksBuffer.size() > 1) {
                auto ticks = 0;
                for (const auto& tick : ticksBuffer) {
                    ticks += tick->value;
                }
                autominy_msgs::Speed speedMsg;
                speedMsg.header.stamp = ros::Time::now();
                speedMsg.header.frame_id = "base_link";
                auto duration = ticksBuffer.size() * 0.01;
                if (duration == 0.0) {
                    return;
                }
                speedMsg.value =
                        ((static_cast<double>(ticks) * config.ticks_to_m) / duration) *
                        static_cast<double>(direction);
                calibratedSpeedPublisher.publish(speedMsg);
            }
        }

        void HardwareCalibrationNodelet::onSteeringFeedback(const autominy_msgs::SteeringFeedbackConstPtr& msg) {
            auto steeringFeedback = msg->value;
            steeringFeedbackBuffer.push_back(steeringFeedback);
            double average = std::accumulate(steeringFeedbackBuffer.begin(), steeringFeedbackBuffer.end(), 0.0) / steeringFeedbackBuffer.size();

            if (average < config.minimum_steering_feedback || average > config.maximum_steering_feedback) {
                ROS_ERROR("Steering feedback is not within range %i to %i: %f. Clamping!", config.minimum_steering_feedback, config.maximum_steering_feedback, average);
                boost::algorithm::clamp(average, config.minimum_steering_feedback, config.maximum_steering_feedback);
            }

            auto radianSteering = mapRange(config.minimum_steering_feedback, config.maximum_steering_feedback, config.minimum_steering_radians, config.maximum_steering_radians, steeringFeedback);

            autominy_msgs::SteeringAngle steeringAngleMsg;
            steeringAngleMsg.header.stamp = ros::Time::now();
            steeringAngleMsg.header.frame_id = "base_link";
            steeringAngleMsg.value = radianSteering;
            steeringAnglePublisher.publish(steeringAngleMsg);
        }

        void HardwareCalibrationNodelet::onWantedSpeed(const autominy_msgs::NormalizedSpeedCommandConstPtr& msg) {
            auto wantedSpeed = msg->value;

            if (wantedSpeed < 0.0) {
                direction = Direction::BACKWARD;
            } else if (wantedSpeed > 0.0) {
                direction = Direction::FORWARD;
            }

            if (wantedSpeed < -1.0 || wantedSpeed > 1.0) {
                ROS_ERROR("Wanted speed is not within range -1.0 to 1.0: %f. Clamping!", wantedSpeed);
                boost::algorithm::clamp(wantedSpeed, -1.0, 1.0);
            }

            auto pwm = mapRange(-1.0, 1.0, config.minimum_speed_pwm, config.maximum_speed_pwm, wantedSpeed);

            autominy_msgs::SpeedCommand speedMsg;
            speedMsg.header.stamp = ros::Time::now();
            speedMsg.header.frame_id = "base_link";
            speedMsg.value = static_cast<int16_t>(pwm);
            speedPublisher.publish(speedMsg);
        }

        void HardwareCalibrationNodelet::onWantedSteering(const autominy_msgs::NormalizedSteeringCommandConstPtr& msg) {
            auto wantedSteering = msg->value;

            if (wantedSteering < -1.0 || wantedSteering > 1.0) {
                ROS_ERROR("Wanted steering is not within range -1.0 to 1.0: %f. Clamping!", wantedSteering);
                boost::algorithm::clamp(wantedSteering, -1.0, 1.0);
            }

            // left and right have different limits
            auto pwm = 0.0;
            if (wantedSteering < 0) {
                pwm = mapRange(-1.0, 0, config.maximum_steering_radians, 0, wantedSteering);
            } else {
                pwm = mapRange(1.0, 0, config.minimum_steering_radians, 0, wantedSteering);
            }
            pwm = mapRange(config.minimum_steering_radians, config.maximum_steering_radians, config.minimum_steering_pwm, config.maximum_steering_pwm, pwm);

            autominy_msgs::SteeringCommand steeringMsg;
            steeringMsg.header.stamp = ros::Time::now();
            steeringMsg.header.frame_id = "base_link";
            steeringMsg.value = static_cast<int16_t>(pwm);
            steeringPublisher.publish(steeringMsg);
        }

        void HardwareCalibrationNodelet::onReconfigure(HardwareCalibrationConfig& config, uint32_t level) {
            this->config = config;
            ticksBuffer.set_capacity(static_cast<unsigned long>(config.number_of_ticks_filter));
            steeringFeedbackBuffer.set_capacity(static_cast<unsigned long>(config.number_of_steering_msgs_filter));
        }

        double HardwareCalibrationNodelet::mapRange(double a1, double a2, double b1, double b2, double s) {
            return b1 + (s-a1) * (b2-b1) / (a2-a1);
        }
};