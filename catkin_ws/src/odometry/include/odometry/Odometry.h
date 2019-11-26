#pragma once

#include <odometry/OdometryConfig.h>
#include <nav_msgs/Odometry.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringAngle.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry {

/** Odometry class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class Odometry {
    public:
        /** Constructor.
         */
        Odometry();

        /** Destructor.
         */
        virtual ~Odometry();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(odometry::OdometryConfig &config);

        void setSpeed(const autominy_msgs::SpeedConstPtr &speed);

        void setSteering(const autominy_msgs::SteeringAngleConstPtr &steering);

        nav_msgs::Odometry step(const ros::TimerEvent& evnt);

    private:
        /// dynamic config attribute
        odometry::OdometryConfig config;
        double currentSpeed = 0.0;
        double currentSteering = 0.0;
        double x, y, yaw;

        tf2_ros::TransformBroadcaster tfBroadCaster;
    };
}
