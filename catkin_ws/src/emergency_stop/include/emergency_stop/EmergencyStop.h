#pragma once

#include <emergency_stop/EmergencyStopConfig.h>
#include <sensor_msgs/LaserScan.h>
#include "autominy_msgs/msg/speed.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"

namespace emergency_stop {

/** EmergencyStop class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class EmergencyStop {
    public:
        /** Constructor.
         */
        EmergencyStop();

        /** Destructor.
         */
        virtual ~EmergencyStop();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(emergency_stop::EmergencyStopConfig &config);

        void checkEmergencyStop(const sensor_msgs::msg::LaserScanConstPtr &scan);

        void setCurrentSpeed(const autominy_msgs::msg::SpeedConstPtr &speed);

        void setWantedSpeed(const autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr &speed);

        autominy_msgs::msg::SpeedPWMCommand getSafeSpeed();

    private:
        /// dynamic config attribute
        emergency_stop::EmergencyStopConfig config;
        double currentSpeed = 0.0;
        int16_t wantedSpeed = 0;
        bool emergencyStop = true;
    };
}
