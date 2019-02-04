#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <emergency_stop/EmergencyStopFwd.h>
#include <emergency_stop/EmergencyStopConfig.h>
#include <emergency_stop/EmergencyStop.h>
#include <sensor_msgs/LaserScan.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/Speed.h>

namespace emergency_stop {

/** EmergencyStop nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
class EmergencyStopNodelet : public nodelet::Nodelet {
   public:
    /** Constructor.
     */
    EmergencyStopNodelet() { }

    /** Destructor.
     */
    virtual ~EmergencyStopNodelet() {}

    /** Nodelet initialization. Called by nodelet manager on initialization,
     ** can be used to e.g. subscribe to topics and define publishers.
     */
    virtual void onInit() override {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        emergencyStop = std::make_shared<EmergencyStop>();

        speedPublisher = nh.advertise<autominy_msgs::SpeedCommand>("speed", 1);
        scanSubscriber = nh.subscribe("scan", 1, &EmergencyStopNodelet::onScan, this);
        wantedSpeedSubscriber = nh.subscribe("wanted_speed", 1, &EmergencyStopNodelet::onWantedSpeed, this);
        currentSpeedSubscriber = nh.subscribe("carstate/speed", 1, &EmergencyStopNodelet::onCurrentSpeed, this);

        config_server_ = boost::make_shared<dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig> >(pnh);
        dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig>::CallbackType f;
        f = boost::bind(&EmergencyStopNodelet::callbackReconfigure, this, _1, _2);
        config_server_->setCallback(f);
    }

   private:
    /** Callback for messages of some type.
     **
     ** @param msg
     */
    void onScan(sensor_msgs::LaserScanConstPtr const & msg) {
        emergencyStop->checkEmergencyStop(msg);
    }

    void onCurrentSpeed(autominy_msgs::SpeedConstPtr const & msg) {
        emergencyStop->setCurrentSpeed(msg);
    }

    void onWantedSpeed(autominy_msgs::SpeedCommandConstPtr const & msg) {
        emergencyStop->setWantedSpeed(msg);
        if (emergencyStop->isEmergencyStop()) {
            autominy_msgs::SpeedCommand emergencyStopMsg;
            emergencyStopMsg.value = 0;
            speedPublisher.publish(emergencyStopMsg);
        } else {
            speedPublisher.publish(msg);
        }
    }

    /** Callback for dynamic_reconfigure.
     **
     ** @param msg
     */
    void callbackReconfigure(emergency_stop::EmergencyStopConfig & config, uint32_t level) {
        emergencyStop->setConfig(config);
    }

    /// subscriber
    ros::Subscriber scanSubscriber;
    ros::Subscriber currentSpeedSubscriber;
    ros::Subscriber wantedSpeedSubscriber;

    /// publisher
    ros::Publisher speedPublisher;

    /// pointer to dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig> > config_server_;

    /// pointer to the functionality class
    EmergencyStopPtr emergencyStop;
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(emergency_stop::EmergencyStopNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
