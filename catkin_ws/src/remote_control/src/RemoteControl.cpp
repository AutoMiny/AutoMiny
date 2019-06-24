#include <remote_control/RemoteControl.h>
#include <ros/ros.h>

namespace remote_control {
    RemoteControl::RemoteControl() : initialized(false), currentSpeed(0), currentSteering(0) {}

    RemoteControl::~RemoteControl() {
        for(const auto& controller : controllers) {
            SDL_GameControllerClose(controller);
        }

        SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
        SDL_Quit();
    }

    void RemoteControl::setConfig(const remote_control::RemoteControlConfig& config) {
        this->config = config;
    }

    bool RemoteControl::checkInput() {
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
                        ROS_INFO("Controller %s connected", SDL_GameControllerName(controller));
                    }
                    break;
                }
                case SDL_CONTROLLERDEVICEREMOVED: {
                    auto con = SDL_GameControllerFromInstanceID(event.cdevice.which);
                    controllers.erase(std::remove(controllers.begin(), controllers.end(), con), controllers.end());
                    ROS_INFO("Controller %s disconnected", SDL_GameControllerName(con));
                    SDL_GameControllerClose(con);
                    break;
                }
                case SDL_CONTROLLERAXISMOTION: {
                    if (std::abs(event.caxis.value) < config.deadspot) {
                        event.caxis.value = 0;
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

    double RemoteControl::getSteering() {
        return (currentSteering / -32767.0) * config.max_steering;
    }

    double RemoteControl::getSpeed() {
        return (currentSpeed / -32767.0) * config.max_speed;
    }
}
