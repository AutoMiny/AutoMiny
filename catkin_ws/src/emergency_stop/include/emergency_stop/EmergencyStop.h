#pragma once

#include <emergency_stop/EmergencyStopConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SpeedCommand.h>

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

        void checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan);

        void setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed);

        void setWantedSpeed(const autominy_msgs::SpeedCommandConstPtr &speed);

        autominy_msgs::SpeedCommand getSafeSpeed();

    private:
        /// dynamic config attribute
        emergency_stop::EmergencyStopConfig config;
        double currentSpeed = 0.0;
        int16_t wantedSpeed = 0;
        bool emergencyStop = true;

        double getBreakDistance();

        double calculateBreakDistance() const;

        bool movingForwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                     float angleIncrement);

        bool movingBackwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                      float angleIncrement);

        bool firstForwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                    float angleIncrement, double angleFront) const;

        bool secondForwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                     float angleIncrement, double angleFront) const;
    };
}
