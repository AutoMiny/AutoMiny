#pragma once

#include <tick_calibration/TickCalibrationConfig.h>
#include <nav_msgs/Odometry.h>
#include "autominy_msgs/msg/tick.hpp"
#include <boost/circular_buffer.hpp>

namespace tick_calibration {

/** TickCalibration class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class TickCalibration {
    public:
        /** Constructor.
         */
        TickCalibration();

        /** Destructor.
         */
        virtual ~TickCalibration();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(tick_calibration::TickCalibrationConfig &config);

        void addLocalization(const nav_msgs::OdometryConstPtr& odom);

        void addTick(const autominy_msgs::msg::TickConstPtr& tick);

        double calibrate();

    private:
        /// dynamic config attribute
        tick_calibration::TickCalibrationConfig config;
        boost::circular_buffer<nav_msgs::OdometryConstPtr> pastTrajectory;
        boost::circular_buffer<autominy_msgs::msg::TickConstPtr> pastTicks;
    };
}
