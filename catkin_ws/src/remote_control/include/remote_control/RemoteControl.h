#pragma once

#include <remote_control/RemoteControlConfig.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>

namespace remote_control {

/** RemoteControl class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class RemoteControl {
    public:
        /** Constructor.
         */
        RemoteControl();

        /** Destructor.
         */
        virtual ~RemoteControl();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(const remote_control::RemoteControlConfig& config);

        bool checkInput();

        double getSteering();

        double getSpeed();
    private:
        /// dynamic config attribute
        remote_control::RemoteControlConfig config;
        std::vector<SDL_GameController*> controllers;
        bool initialized;

        int currentSteering;
        int currentSpeed;
    };
}
