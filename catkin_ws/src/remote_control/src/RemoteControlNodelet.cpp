#include "remote_control/RemoteControl.h"

namespace remote_control {

    RemoteControlNodelet::~RemoteControlNodelet() {
        for(const auto& controller : controllers) {
            SDL_GameControllerClose(controller);
        }

        SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
        SDL_Quit();
    }

    RemoteControlNodelet::RemoteControlNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("remote_control", opts),
                                                                                  initialized(false), currentSpeed(0), currentSteering(0) {
        config.max_speed = declare_parameter<double>("max_speed", 0.2);
        config.max_steering = declare_parameter<double>("max_steering", 1.0);
        config.deadspot = declare_parameter<int>("deadspot", 6000);

        speedPublisher = create_publisher<autominy_msgs::msg::NormalizedSpeedCommand>("speed", 1);
        steeringPublisher = create_publisher<autominy_msgs::msg::NormalizedSteeringCommand>("steering", 1);

        inputTimer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.002), std::bind(&RemoteControlNodelet::onCheckInput, this));
        publishTimer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&RemoteControlNodelet::onPublish, this));
    }

    void RemoteControlNodelet::onCheckInput() {
        checkInput();
    }

    void RemoteControlNodelet::onPublish() {
        autominy_msgs::msg::NormalizedSteeringCommand steeringCommand;
        autominy_msgs::msg::NormalizedSpeedCommand speedCommand;

        steeringCommand.header.stamp = speedCommand.header.stamp = now();
        steeringCommand.header.frame_id = speedCommand.header.frame_id = "base_link";

        steeringCommand.value = getSteering();
        speedCommand.value = getSpeed();

        steeringPublisher->publish(steeringCommand);
        speedPublisher->publish(speedCommand);
    }

    double RemoteControlNodelet::getSteering() {
        return std::copysign(std::clamp((currentSteering / -32767.0), -config.max_steering, config.max_steering), -currentSteering);
    }

    double RemoteControlNodelet::getSpeed() {
        return std::copysign(std::clamp((currentSpeed / -32767.0), -config.max_speed, config.max_speed), -currentSpeed);
    }

    bool RemoteControlNodelet::checkInput() {
        // Calls to SDL should come from the same thread so initializing inside the constructor is not possible
        if (!initialized) {
            SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
            SDL_Init(SDL_INIT_GAMECONTROLLER);
            initialized = true;
        }

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_CONTROLLERDEVICEADDED: {
                    if (SDL_IsGameController(event.cdevice.which)) {
                        auto controller = SDL_GameControllerOpen(event.cdevice.which);
                        controllers.emplace_back(controller);
                        RCLCPP_INFO(get_logger(), "Controller %s connected", SDL_GameControllerName(controller));
                    }
                    break;
                }
                case SDL_CONTROLLERDEVICEREMOVED: {
                    auto con = SDL_GameControllerFromInstanceID(event.cdevice.which);
                    controllers.erase(std::remove(controllers.begin(), controllers.end(), con), controllers.end());
                    RCLCPP_INFO(get_logger(), "Controller %s disconnected", SDL_GameControllerName(con));
                    SDL_GameControllerClose(con);
                    break;
                }
                case SDL_CONTROLLERAXISMOTION: {
                    if (std::abs(event.caxis.value) < config.deadspot) {
                        if (lastValue > 0) {
                            event.caxis.value = 1;
                        }
                        if (lastValue < 0) {
                            event.caxis.value = -1;
                        }
                    } else {
                        lastValue = event.caxis.value;
                    }

                    // Speed
                    if (event.caxis.axis == SDL_GameControllerAxis::SDL_CONTROLLER_AXIS_LEFTY) {
                        currentSpeed = event.caxis.value;
                    }

                    // Steering
                    if (event.caxis.axis == SDL_GameControllerAxis::SDL_CONTROLLER_AXIS_RIGHTX) {
                        currentSteering = event.caxis.value;
                    }

                    return true;
                }
            }
        }

        return false;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(remote_control::RemoteControlNodelet)