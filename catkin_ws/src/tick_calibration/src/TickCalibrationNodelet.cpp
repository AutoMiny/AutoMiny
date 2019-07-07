#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <tick_calibration/TickCalibrationFwd.h>
#include <tick_calibration/TickCalibrationConfig.h>
#include <tick_calibration/TickCalibration.h>
#include <nav_msgs/Odometry.h>
#include <autominy_msgs/Tick.h>

namespace tick_calibration {

/** TickCalibration nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class TickCalibrationNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        TickCalibrationNodelet() = default;

        /** Destructor.
         */
        ~TickCalibrationNodelet() override {}

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        virtual void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            tickCalibration = std::make_shared<TickCalibration>();

            localizationSubscriber = pnh.subscribe("/sensors/localization/filtered_map", 10, &TickCalibrationNodelet::onLocalization, this, ros::TransportHints().tcpNoDelay());
            tickSubscriber = pnh.subscribe("/sensors/arduino/ticks", 10, &TickCalibrationNodelet::onTick, this, ros::TransportHints().tcpNoDelay());
            calibrationTimer = pnh.createTimer(ros::Duration(0.1), &TickCalibrationNodelet::onCalibrate, this);

            config_server_ = boost::make_shared<dynamic_reconfigure::Server<tick_calibration::TickCalibrationConfig> >(pnh);
            dynamic_reconfigure::Server<tick_calibration::TickCalibrationConfig>::CallbackType f;
            f = boost::bind(&TickCalibrationNodelet::callbackReconfigure, this, _1, _2);
            config_server_->setCallback(f);
        }

    private:
        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(tick_calibration::TickCalibrationConfig &config, uint32_t level) {
            tickCalibration->setConfig(config);
            calibrationTimer.setPeriod(ros::Duration(1.0 / config.calibration_frequency));
        }

        void onLocalization(const nav_msgs::OdometryConstPtr& odom) {
            tickCalibration->addLocalization(odom);
        }

        void onTick(const autominy_msgs::TickConstPtr& tick) {
            tickCalibration->addTick(tick);
        }

        void onCalibrate(const ros::TimerEvent& evnt) {
            tickCalibration->calibrate();
        }

        /// subscriber
        ros::Subscriber localizationSubscriber;
        ros::Subscriber tickSubscriber;
        ros::Timer calibrationTimer;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<tick_calibration::TickCalibrationConfig> > config_server_;

        /// pointer to the functionality class
        TickCalibrationPtr tickCalibration;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tick_calibration::TickCalibrationNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
