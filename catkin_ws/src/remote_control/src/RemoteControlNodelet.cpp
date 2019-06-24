#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <remote_control/RemoteControlConfig.h>
#include <autominy_msgs/NormalizedSteeringCommand.h>
#include <autominy_msgs/NormalizedSpeedCommand.h>

#include <remote_control/RemoteControl.h>

namespace remote_control {

/** RemoteControl nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class RemoteControlNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        RemoteControlNodelet() = default;

        /** Destructor.
         */
        ~RemoteControlNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();
            remoteControl = std::make_unique<RemoteControl>();

            speedPublisher = pnh.advertise<autominy_msgs::NormalizedSpeedCommand>("speed", 1);
            steeringPublisher = pnh.advertise<autominy_msgs::NormalizedSteeringCommand>("steering", 1);

            config_server_ = boost::make_shared<dynamic_reconfigure::Server<remote_control::RemoteControlConfig> >(pnh);
            dynamic_reconfigure::Server<remote_control::RemoteControlConfig>::CallbackType f;
            f = boost::bind(&RemoteControlNodelet::callbackReconfigure, this, _1, _2);
            config_server_->setCallback(f);

            inputTimer = pnh.createTimer(ros::Duration(0.002), &RemoteControlNodelet::onCheckInput, this);
            publishTimer = pnh.createTimer(ros::Duration(0.1), &RemoteControlNodelet::onPublish, this);
        }

    private:
        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(const remote_control::RemoteControlConfig &config, uint32_t level) {
            remoteControl->setConfig(config);
        }

        void onCheckInput(const ros::TimerEvent& event) {
            if (remoteControl->checkInput()) {
            }
        }

        void onPublish(const ros::TimerEvent& event) {
            autominy_msgs::NormalizedSteeringCommand steeringCommand;
            autominy_msgs::NormalizedSpeedCommand speedCommand;

            steeringCommand.value = remoteControl->getSteering();
            speedCommand.value = remoteControl->getSpeed();

            steeringPublisher.publish(steeringCommand);
            speedPublisher.publish(speedCommand);
        }

        /// timer
        ros::Timer inputTimer;
        ros::Timer publishTimer;

        /// publisher
        ros::Publisher speedPublisher;
        ros::Publisher steeringPublisher;

        std::unique_ptr<RemoteControl> remoteControl;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<remote_control::RemoteControlConfig> > config_server_;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(remote_control::RemoteControlNodelet, nodelet::Nodelet)
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
