#pragma once

#include "rclcpp/rclcpp.hpp"

#include "autominy_msgs/msg/normalized_steering_command.hpp"
#include "autominy_msgs/msg/normalized_speed_command.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>
#include <algorithm>


namespace remote_control {

    struct RemoteControlConfig {
        double max_speed = 0.2;
        double max_steering = 1.0;
        int deadspot = 6000;
    };

/** RemoteControl nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class RemoteControlNodelet : public rclcpp::Node {
    public:
        /** Destructor.
         */
        ~RemoteControlNodelet();;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        RemoteControlNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:
        void onCheckInput();

        void onPublish();

        double getSteering();

        double getSpeed();

        bool checkInput();


        /// timer
        rclcpp::TimerBase::SharedPtr inputTimer;
        rclcpp::TimerBase::SharedPtr publishTimer;

        /// publisher
        rclcpp::Publisher<autominy_msgs::msg::NormalizedSpeedCommand>::SharedPtr speedPublisher;
        rclcpp::Publisher<autominy_msgs::msg::NormalizedSteeringCommand>::SharedPtr steeringPublisher;

        RemoteControlConfig config;

        std::vector<SDL_GameController*> controllers;
        bool initialized{};

        int currentSteering{};
        int currentSpeed{};
        int lastValue{};

    };
}