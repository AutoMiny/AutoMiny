#pragma once

#include <emergency_stop/EmergencyStopConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <autominy_msgs/Speed.h>

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
    void setConfig(emergency_stop::EmergencyStopConfig & config);

    void checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan);

    void setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed);

    bool isEmergencyStop();
   private:
    /// dynamic config attribute
    emergency_stop::EmergencyStopConfig config;
    double currentSpeed = 0.0;
    bool emergencyStop = true;
};
}
